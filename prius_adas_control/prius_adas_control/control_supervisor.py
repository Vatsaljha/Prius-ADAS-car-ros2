#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float64
from prius_msgs.msg import Control


class ControlSupervisor(Node):

    def __init__(self):
        super().__init__('control_supervisor')

        # =========================================================
        # PUBLISHERS
        # =========================================================

        # The supervisor is the ONLY node that should publish
        # the final Prius control command.
        self.control_pub = self.create_publisher(
            Control,
            '/prius/control',
            10
        )

        # Current control authority:
        # LANE / NAV2
        self.mode_pub = self.create_publisher(
            String,
            '/adas/control_mode',
            10
        )

        # =========================================================
        # LANE CONTROL INPUT
        # =========================================================

        self.lane_sub = self.create_subscription(
            Control,
            '/adas/lane_control',
            self.lane_control_callback,
            10
        )

        # =========================================================
        # NAV2 CONTROL INPUT
        # =========================================================

        self.nav2_sub = self.create_subscription(
            Control,
            '/adas/nav2_control',
            self.nav2_control_callback,
            10
        )

        # =========================================================
        # LANE HEALTH INPUT
        # =========================================================

        self.lane_valid_sub = self.create_subscription(
            Bool,
            '/adas/lane_valid',
            self.lane_valid_callback,
            10
        )

        # =========================================================
        # OBSTACLE INPUT
        # =========================================================

        self.obstacle_detected_sub = self.create_subscription(
            Bool,
            '/adas/obstacle_detected',
            self.obstacle_detected_callback,
            10
        )

        # =========================================================
        # FINAL EMERGENCY BRAKE INPUT
        # =========================================================

        self.brake_sub = self.create_subscription(
            Float64,
            '/adas/brake',
            self.brake_callback,
            10
        )

        # =========================================================
        # COMMAND STATE
        # =========================================================

        self.latest_lane_control = None
        self.latest_nav2_control = None

        # =========================================================
        # FINAL BRAKE STATE
        # =========================================================

        self.emergency_brake = 0.0

        # =========================================================
        # NAV2 COMMAND GENERATION
        # =========================================================

        # Used to distinguish a genuinely new Nav2 command from a
        # stale command that existed before the obstacle appeared.
        self.nav2_command_generation = 0

        # =========================================================
        # OBSTACLE HANDOVER STATE
        # =========================================================

        self.obstacle_takeover_active = False
        self.obstacle_takeover_start_time = None
        self.nav2_generation_at_obstacle = 0

        # Minimum safety-brake period after an obstacle-triggered
        # takeover. This is time-based, not distance-based.
        self.obstacle_takeover_brake_hold = 0.50

        # =========================================================
        # LANE HEALTH STATE
        # =========================================================

        self.lane_valid = False

        # =========================================================
        # OBSTACLE STATE
        # =========================================================

        self.obstacle_detected = False

        # =========================================================
        # TIMESTAMPS
        # =========================================================

        now = self.get_clock().now()

        self.last_lane_time = now
        self.last_nav2_time = now
        self.last_lane_valid_time = now
        self.last_obstacle_time = now
        self.last_brake_time = now

        # Start time of temporary lane loss.
        self.lane_loss_start_time = None

        # =========================================================
        # TIMEOUTS
        # =========================================================

        # Lane command must remain fresh.
        self.lane_timeout = 0.30

        # Nav2 command must remain fresh.
        self.nav2_timeout = 0.50

        # Lane-health message timeout.
        self.lane_valid_timeout = 0.30

        # Obstacle detector timeout.
        self.obstacle_timeout = 0.50

        # =========================================================
        # LANE LOSS GRACE
        # =========================================================

        # Normal temporary lane-loss grace.
        self.lane_loss_grace_period = 0.80

        # Larger grace during strong right turns.
        self.right_turn_grace_period = 1.80

        # Negative steering = right turn.
        self.right_turn_steer_threshold = -0.30

        # =========================================================
        # LANE RECOVERY
        # =========================================================

        # Require several healthy cycles before returning from
        # Nav2 to lane control.
        self.lane_recovery_required = 10

        self.lane_recovery_count = 0

        # =========================================================
        # CURRENT AUTHORITY
        # =========================================================

        # Start conservatively in NAV2.
        self.mode = 'NAV2'

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        # =========================================================
        # STARTUP LOG
        # =========================================================

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'CONTROL SUPERVISOR STARTED'
        )

        self.get_logger().info(
            'Lane input    : /adas/lane_control'
        )

        self.get_logger().info(
            'Nav2 input    : /adas/nav2_control'
        )

        self.get_logger().info(
            'Lane health   : /adas/lane_valid'
        )

        self.get_logger().info(
            'Obstacle      : /adas/obstacle_detected'
        )

        self.get_logger().info(
            'Emergency brake: /adas/brake'
        )

        self.get_logger().info(
            f'Obstacle takeover brake hold: '
            f'{self.obstacle_takeover_brake_hold:.2f} s'
        )

        self.get_logger().info(
            'Final output  : /prius/control'
        )

        self.get_logger().info(
            f'Normal lane-loss grace: '
            f'{self.lane_loss_grace_period:.2f} s'
        )

        self.get_logger().info(
            f'Right-turn lane-loss grace: '
            f'{self.right_turn_grace_period:.2f} s'
        )

        self.get_logger().info(
            f'Lane recovery cycles: '
            f'{self.lane_recovery_required}'
        )

        self.get_logger().info(
            'Initial mode: NAV2'
        )

        self.get_logger().info(
            '=========================================='
        )

    # =============================================================
    # LANE CONTROL CALLBACK
    # =============================================================

    def lane_control_callback(self, msg):

        self.latest_lane_control = msg

        self.last_lane_time = (
            self.get_clock().now()
        )

    # =============================================================
    # NAV2 CONTROL CALLBACK
    # =============================================================

    def nav2_control_callback(self, msg):

        self.latest_nav2_control = msg

        self.nav2_command_generation += 1

        self.last_nav2_time = (
            self.get_clock().now()
        )

    # =============================================================
    # LANE VALID CALLBACK
    # =============================================================

    def lane_valid_callback(self, msg):

        self.lane_valid = bool(
            msg.data
        )

        self.last_lane_valid_time = (
            self.get_clock().now()
        )

    # =============================================================
    # OBSTACLE CALLBACK
    # =============================================================

    def obstacle_detected_callback(self, msg):

        self.obstacle_detected = bool(
            msg.data
        )

        self.last_obstacle_time = (
            self.get_clock().now()
        )

    # =============================================================
    # EMERGENCY BRAKE CALLBACK
    # =============================================================

    def brake_callback(self, msg):

        try:
            self.emergency_brake = float(msg.data)
        except Exception:
            self.emergency_brake = 0.0

        self.last_brake_time = (
            self.get_clock().now()
        )

    # =============================================================
    # STOP COMMAND
    # =============================================================

    def stop_command(self):

        msg = Control()

        msg.throttle = 0.0
        msg.brake = 1.0
        msg.steer = 0.0
        msg.shift_gears = Control.FORWARD

        return msg

    # =============================================================
    # TIME HELPER
    # =============================================================

    def age(self, stamp):

        return (
            self.get_clock().now()
            -
            stamp
        ).nanoseconds / 1e9

    # =============================================================
    # RIGHT TURN DETECTION
    # =============================================================

    def is_strong_right_turn(self):

        if self.latest_lane_control is None:
            return False

        return (
            self.latest_lane_control.steer
            <=
            self.right_turn_steer_threshold
        )

    # =============================================================
    # CURRENT LANE GRACE PERIOD
    # =============================================================

    def get_lane_loss_grace_period(self):

        if self.is_strong_right_turn():

            return self.right_turn_grace_period

        return self.lane_loss_grace_period

    # =============================================================
    # LANE HEALTH CHECK
    # =============================================================

    def lane_is_healthy(self):

        lane_age = self.age(
            self.last_lane_time
        )

        health_age = self.age(
            self.last_lane_valid_time
        )

        health_message_available = (
            health_age
            <=
            self.lane_valid_timeout
        )

        lane_command_available = (
            self.latest_lane_control
            is not None
        )

        lane_command_recent = (
            lane_age
            <=
            self.lane_timeout
        )

        return (
            health_message_available
            and
            lane_command_available
            and
            lane_command_recent
            and
            self.lane_valid
        )

    # =============================================================
    # NAV2 HEALTH CHECK
    # =============================================================

    def nav2_is_healthy(self):

        nav2_age = self.age(
            self.last_nav2_time
        )

        return (
            self.latest_nav2_control
            is not None
            and
            nav2_age
            <=
            self.nav2_timeout
        )

    # =============================================================
    # OBSTACLE DATA HEALTH
    # =============================================================

    def obstacle_data_is_fresh(self):

        obstacle_age = self.age(
            self.last_obstacle_time
        )

        return (
            obstacle_age
            <=
            self.obstacle_timeout
        )

    # =============================================================
    # OBSTACLE STATE
    # =============================================================

    def obstacle_is_active(self):

        # Fresh TRUE signal = active obstacle.
        if self.obstacle_data_is_fresh():

            return self.obstacle_detected

        # No fresh obstacle signal.
        return False

    # =============================================================
    # EMERGENCY BRAKE HEALTH
    # =============================================================

    def emergency_brake_is_active(self):

        brake_age = self.age(
            self.last_brake_time
        )

        return (
            brake_age <= self.obstacle_timeout
            and
            self.emergency_brake >= 0.5
        )

    # =============================================================
    # OBSTACLE TAKEOVER READINESS
    # =============================================================

    def nav2_has_fresh_command_after_obstacle(self):

        return (
            self.nav2_command_generation
            >
            self.nav2_generation_at_obstacle
        )

    def obstacle_takeover_brake_is_active(self, now):

        if not self.obstacle_takeover_active:
            return False

        # Never release the initial safety stop until at least the
        # minimum handover brake hold has elapsed.
        if self.obstacle_takeover_start_time is None:
            return True

        elapsed = (
            now
            -
            self.obstacle_takeover_start_time
        ).nanoseconds / 1e9

        if (
            elapsed
            <
            self.obstacle_takeover_brake_hold
        ):
            return True

        # Do not release the brake onto a stale Nav2 command.
        if not self.nav2_has_fresh_command_after_obstacle():
            return True

        return False

    # =============================================================
    # MODE PUBLISH
    # =============================================================

    def publish_mode(self):

        msg = String()

        msg.data = self.mode

        self.mode_pub.publish(
            msg
        )

    # =============================================================
    # MAIN CONTROL LOGIC
    # =============================================================

    def update(self):

        now = self.get_clock().now()

        lane_healthy = (
            self.lane_is_healthy()
        )

        nav2_healthy = (
            self.nav2_is_healthy()
        )

        obstacle_active = (
            self.obstacle_is_active()
        )

        lane_age = self.age(
            self.last_lane_time
        )

        # =====================================================
        # OBSTACLE HAS PRIORITY
        #
        # The detector itself decides when an obstacle exists.
        # No obstacle distance is hardcoded here.
        #
        # Nav2 then receives control and performs its own
        # costmap/path replanning.
        # =====================================================

        if obstacle_active:

            self.lane_recovery_count = 0
            self.lane_loss_start_time = None

            # Arm the safety brake exactly when the supervisor
            # first reacts to the obstacle. The previous Nav2
            # command is deliberately NOT allowed through.
            if not self.obstacle_takeover_active:

                self.obstacle_takeover_active = True
                self.obstacle_takeover_start_time = now
                self.nav2_generation_at_obstacle = (
                    self.nav2_command_generation
                )

                self.get_logger().warn(
                    'OBSTACLE DETECTED '
                    '-> IMMEDIATE BRAKE '
                    '-> NAV2 TAKEOVER'
                )

            self.mode = 'NAV2'

        # =====================================================
        # LANE MODE
        # =====================================================

        elif self.mode == 'LANE':

            # -------------------------------------------------
            # Lane healthy.
            # -------------------------------------------------

            if lane_healthy:

                self.lane_loss_start_time = None

            # -------------------------------------------------
            # Lane temporarily unhealthy.
            # -------------------------------------------------

            else:

                # -------------------------------------------------
                # Hard command timeout.
                #
                # If lane controller stops publishing, do not
                # wait for the curve grace period.
                # -------------------------------------------------

                if (
                    self.latest_lane_control
                    is None
                    or
                    lane_age
                    >
                    self.lane_timeout
                ):

                    self.lane_loss_start_time = None

                    self.mode = 'NAV2'

                    self.lane_recovery_count = 0

                    self.get_logger().warn(
                        'LANE COMMAND TIMEOUT '
                        '-> NAV2 TAKEOVER'
                    )

                else:

                    # -------------------------------------------------
                    # Temporary perception loss.
                    # -------------------------------------------------

                    if (
                        self.lane_loss_start_time
                        is None
                    ):

                        self.lane_loss_start_time = now

                        grace = (
                            self.get_lane_loss_grace_period()
                        )

                        if self.is_strong_right_turn():

                            self.get_logger().info(
                                'RIGHT TURN DETECTED - '
                                f'LANE GRACE {grace:.2f} s'
                            )

                    lane_loss_duration = (
                        now
                        -
                        self.lane_loss_start_time
                    ).nanoseconds / 1e9

                    current_grace = (
                        self.get_lane_loss_grace_period()
                    )

                    if (
                        lane_loss_duration
                        >=
                        current_grace
                    ):

                        self.mode = 'NAV2'

                        self.lane_recovery_count = 0

                        self.get_logger().warn(
                            'LANE LOST AFTER GRACE PERIOD '
                            '-> NAV2 TAKEOVER'
                        )

        # =====================================================
        # NAV2 MODE
        # =====================================================

        elif self.mode == 'NAV2':

            # -------------------------------------------------
            # Obstacle still active.
            #
            # Stay with Nav2.
            # -------------------------------------------------

            if obstacle_active:

                self.lane_recovery_count = 0

            # -------------------------------------------------
            # No obstacle active.
            #
            # Normal lane recovery may begin.
            # -------------------------------------------------

            else:

                if lane_healthy:

                    self.lane_recovery_count += 1

                    if (
                        self.lane_recovery_count
                        >=
                        self.lane_recovery_required
                    ):

                        self.mode = 'LANE'

                        self.lane_recovery_count = 0

                        self.lane_loss_start_time = None

                        self.get_logger().info(
                            'LANE RECOVERED -> '
                            'LANE CONTROL RESTORED'
                        )

                        self.obstacle_takeover_active = False
                        self.obstacle_takeover_start_time = None
                        self.nav2_generation_at_obstacle = (
                            self.nav2_command_generation
                        )

                else:

                    self.lane_recovery_count = 0

        # =====================================================
        # UNKNOWN MODE
        # =====================================================

        else:

            self.mode = 'NAV2'

            self.lane_recovery_count = 0

            self.lane_loss_start_time = None

        # =====================================================
        # SELECT FINAL COMMAND
        # =====================================================

        emergency_brake_active = (
            self.emergency_brake_is_active()
        )

        obstacle_handover_brake_active = (
            self.obstacle_takeover_brake_is_active(now)
        )

        # -----------------------------------------------------
        # SAFETY BRAKE HAS ABSOLUTE FINAL PRIORITY
        # -----------------------------------------------------
        #
        # obstacle detected
        #       -> immediate stop
        #       -> wait for a fresh Nav2 command
        #       -> release brake only after the handover hold
        #
        # This prevents a stale forward Nav2 command from being
        # published during the obstacle handover.
        # -----------------------------------------------------

        if (
            emergency_brake_active
            or
            obstacle_handover_brake_active
        ):

            self.control_pub.publish(
                self.stop_command()
            )

        elif self.mode == 'LANE':

            # Lane controller has authority.
            if self.latest_lane_control is not None:

                self.control_pub.publish(
                    self.latest_lane_control
                )

            else:

                self.control_pub.publish(
                    self.stop_command()
                )

        elif self.mode == 'NAV2':

            # Nav2 has authority only when its command is fresh.
            if nav2_healthy:

                self.control_pub.publish(
                    self.latest_nav2_control
                )

            else:

                # Never use stale lane control while Nav2 is
                # the selected authority.
                self.control_pub.publish(
                    self.stop_command()
                )

        else:

            self.control_pub.publish(
                self.stop_command()
            )

        # -----------------------------------------------------
        # Mark the obstacle handover complete once the obstacle
        # has cleared, the safety hold has elapsed, and a fresh
        # Nav2 command has arrived.
        # -----------------------------------------------------

        if self.obstacle_takeover_active:

            handover_brake_still_active = (
                self.obstacle_takeover_brake_is_active(now)
            )

            if (
                not obstacle_active
                and
                not emergency_brake_active
                and
                not handover_brake_still_active
            ):

                self.obstacle_takeover_active = False
                self.obstacle_takeover_start_time = None
                self.nav2_generation_at_obstacle = (
                    self.nav2_command_generation
                )

                self.get_logger().info(
                    'OBSTACLE HANDOVER COMPLETE '
                    '-> NAV2 COMMAND RELEASED'
                )

        # =====================================================
        # MODE STATUS
        # =====================================================

        self.publish_mode()

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def shutdown_vehicle(self):

        try:

            stop = self.stop_command()

            for _ in range(5):

                self.control_pub.publish(
                    stop
                )

        except Exception:

            pass


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ControlSupervisor()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown_vehicle()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()