#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from prius_msgs.msg import Control


class VehicleController(Node):

    def __init__(self):

        super().__init__("vehicle_controller")

        # =====================================================
        # PRIUS CONTROL OUTPUT
        # =====================================================

        self.control_pub = self.create_publisher(
            Control,
            "/prius/control",
            10,
        )

        # =====================================================
        # LANE DETECTION INPUTS
        # =====================================================

        self.lateral_sub = self.create_subscription(
            Float64,
            "/adas/lane_lateral_error",
            self.lateral_callback,
            10,
        )

        self.heading_sub = self.create_subscription(
            Float64,
            "/adas/lane_heading_error",
            self.heading_callback,
            10,
        )

        # =====================================================
        # BRAKE INPUT
        # =====================================================

        # This will later come from the LiDAR + sonar
        # obstacle detector.
        self.brake_sub = self.create_subscription(
            Float64,
            "/adas/brake",
            self.brake_callback,
            10,
        )

        # =====================================================
        # DEBUG OUTPUTS
        # =====================================================

        self.steering_pub = self.create_publisher(
            Float64,
            "/adas/target_steering",
            10,
        )

        self.speed_pub = self.create_publisher(
            Float64,
            "/adas/target_speed",
            10,
        )

        # =====================================================
        # INPUT STATE
        # =====================================================

        self.lateral_error = 0.0
        self.heading_error = 0.0
        self.brake = 0.0

        self.last_lateral_time = 0.0
        self.last_heading_time = 0.0

        # =====================================================
        # DERIVATIVE MEMORY
        # =====================================================

        self.previous_lateral = 0.0
        self.previous_heading = 0.0

        # =====================================================
        # STEERING GAINS
        # =====================================================

        # This is the better-performing tuning you reached.
        self.kp_lateral = 1.35
        self.kd_lateral = 0.008

        self.kp_heading = 0.48
        self.kd_heading = 0.008

        # =====================================================
        # STEERING SIGN
        # =====================================================

        # Experimentally confirmed for your Prius.
        self.steering_sign = -1.0

        # =====================================================
        # STEERING LIMIT
        # =====================================================

        self.max_steering = 1.0

        # =====================================================
        # STEERING RESPONSE
        # =====================================================

        self.max_steering_rate = 5.0

        self.current_steering = 0.0

        # =====================================================
        # STEERING FILTER
        # =====================================================

        self.steering_alpha = 0.60

        # =====================================================
        # LATERAL AUTHORITY
        # =====================================================

        self.lateral_activation = 0.16
        self.lateral_min_factor = 0.55

        # =====================================================
        # CORNER RESPONSE
        # =====================================================

        self.corner_lateral_threshold = 0.28
        self.corner_heading_threshold = 0.20

        self.corner_gain = 0.35

        # =====================================================
        # HARD CORNER
        # =====================================================

        self.hard_heading_threshold = 0.50
        self.hard_corner_gain = 0.35

        # =====================================================
        # SPEED
        # =====================================================

        self.max_speed = 0.25
        self.medium_speed = 0.12
        self.low_speed = 0.075
        self.minimum_speed = 0.045

        # =====================================================
        # VEHICLE THROTTLE
        # =====================================================

        # IMPORTANT:
        # We keep the safe throttle mapping that solved your
        # original high-speed problem.
        self.max_throttle = 0.06

        self.throttle_rate_up = 0.10
        self.throttle_rate_down = 0.30

        self.current_throttle = 0.0

        # =====================================================
        # TURN THROTTLE REDUCTION
        # =====================================================

        self.minimum_turn_factor = 0.30

        # =====================================================
        # PERCEPTION SAFETY
        # =====================================================

        self.perception_timeout = 0.50

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.loop_hz = 20.0
        self.dt = 1.0 / self.loop_hz

        self.timer = self.create_timer(
            self.dt,
            self.control_loop,
        )

        self.last_time = time.monotonic()

        self.get_logger().info(
            "Combined steering + vehicle controller started"
        )

    # =========================================================
    # LATERAL CALLBACK
    # =========================================================

    def lateral_callback(self, msg):

        self.lateral_error = float(
            msg.data
        )

        self.last_lateral_time = time.monotonic()

    # =========================================================
    # HEADING CALLBACK
    # =========================================================

    def heading_callback(self, msg):

        self.heading_error = float(
            msg.data
        )

        self.last_heading_time = time.monotonic()

    # =========================================================
    # BRAKE CALLBACK
    # =========================================================

    def brake_callback(self, msg):

        self.brake = max(
            0.0,
            min(
                1.0,
                float(
                    msg.data
                ),
            ),
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        now = time.monotonic()

        dt = (
            now
            -
            self.last_time
        )

        if dt <= 0.0:
            dt = self.dt

        if dt > 0.20:
            dt = self.dt

        self.last_time = now

        # =====================================================
        # PERCEPTION TIMEOUT
        # =====================================================

        lateral_age = (
            now
            -
            self.last_lateral_time
        )

        heading_age = (
            now
            -
            self.last_heading_time
        )

        perception_ok = (
            lateral_age <= self.perception_timeout
            and
            heading_age <= self.perception_timeout
        )

        # =====================================================
        # FAIL-SAFE STOP
        # =====================================================

        if not perception_ok:

            self.current_steering = (
                self.rate_limit_steering(
                    0.0,
                    dt,
                )
            )

            self.current_throttle = 0.0

            self.publish_prius_control(
                steering=self.current_steering,
                throttle=0.0,
                brake=1.0,
                gear=Control.NEUTRAL,
            )

            self.publish_debug(
                self.current_steering,
                0.0,
            )

            return

        # =====================================================
        # INPUTS
        # =====================================================

        lateral = max(
            -0.80,
            min(
                0.80,
                float(
                    self.lateral_error
                ),
            ),
        )

        heading = max(
            -0.90,
            min(
                0.90,
                float(
                    self.heading_error
                ),
            ),
        )

        abs_lateral = abs(
            lateral
        )

        abs_heading = abs(
            heading
        )

        # =====================================================
        # DERIVATIVE
        # =====================================================

        lateral_derivative = (
            lateral
            -
            self.previous_lateral
        ) / dt

        heading_derivative = (
            heading
            -
            self.previous_heading
        ) / dt

        self.previous_lateral = lateral
        self.previous_heading = heading

        lateral_derivative = max(
            -2.0,
            min(
                2.0,
                lateral_derivative,
            ),
        )

        heading_derivative = max(
            -2.0,
            min(
                2.0,
                heading_derivative,
            ),
        )

        # =====================================================
        # LATERAL CONTROL
        # =====================================================

        lateral_control = (
            self.kp_lateral
            *
            lateral
        )

        lateral_control += (
            self.kd_lateral
            *
            lateral_derivative
        )

        # =====================================================
        # HEADING CONTROL
        # =====================================================

        heading_control = (
            self.kp_heading
            *
            heading
        )

        heading_control += (
            self.kd_heading
            *
            heading_derivative
        )

        # =====================================================
        # COMBINE
        # =====================================================

        steering_raw = (
            lateral_control
            +
            heading_control
        )

        # =====================================================
        # PRESERVE LATERAL AUTHORITY
        # =====================================================

        if (
            abs_lateral
            >
            self.lateral_activation
        ):

            minimum_required = (
                self.lateral_min_factor
                *
                abs(lateral_control)
            )

            if (
                abs(steering_raw)
                <
                minimum_required
            ):

                direction = (
                    1.0
                    if lateral_control >= 0.0
                    else -1.0
                )

                steering_raw = (
                    direction
                    *
                    minimum_required
                )

        # =====================================================
        # CORNER BOOST
        # =====================================================

        if (
            abs_lateral
            >
            self.corner_lateral_threshold
            and
            abs_heading
            >
            self.corner_heading_threshold
        ):

            direction = (
                1.0
                if lateral_control >= 0.0
                else -1.0
            )

            corner_amount = (
                0.60 * abs_lateral
                +
                0.40 * abs_heading
            )

            steering_raw += (
                direction
                *
                self.corner_gain
                *
                corner_amount
            )

        # =====================================================
        # HARD CORNER BOOST
        # =====================================================

        if (
            abs_heading
            >
            self.hard_heading_threshold
        ):

            direction = (
                1.0
                if lateral_control >= 0.0
                else -1.0
            )

            hard_amount = (
                abs_heading
                -
                self.hard_heading_threshold
            )

            hard_amount = min(
                hard_amount,
                0.40,
            )

            steering_raw += (
                direction
                *
                self.hard_corner_gain
                *
                hard_amount
            )

        # =====================================================
        # APPLY VEHICLE SIGN
        # =====================================================

        steering_target = (
            self.steering_sign
            *
            steering_raw
        )

        # =====================================================
        # LIMIT
        # =====================================================

        steering_target = max(
            -self.max_steering,
            min(
                self.max_steering,
                steering_target,
            ),
        )

        # =====================================================
        # RATE LIMIT
        # =====================================================

        steering_target = (
            self.rate_limit_steering(
                steering_target,
                dt,
            )
        )

        # =====================================================
        # FILTER
        # =====================================================

        steering = (
            self.steering_alpha
            *
            steering_target
            +
            (
                1.0
                -
                self.steering_alpha
            )
            *
            self.current_steering
        )

        self.current_steering = steering

        # =====================================================
        # SPEED
        # =====================================================

        target_speed = self.calculate_speed(
            lateral,
            heading,
            steering,
        )

        # =====================================================
        # BRAKE OVERRIDE
        # =====================================================

        if self.brake > 0.01:

            target_speed = 0.0

        # =====================================================
        # THROTTLE
        # =====================================================

        target_throttle = (
            self.calculate_target_throttle(
                target_speed,
                steering,
            )
        )

        # =====================================================
        # THROTTLE RAMP
        # =====================================================

        self.current_throttle = (
            self.update_throttle(
                target_throttle,
                dt,
            )
        )

        # =====================================================
        # BRAKE
        # =====================================================

        if self.brake > 0.01:

            self.current_throttle = 0.0

        # =====================================================
        # GEAR
        # =====================================================

        if self.brake > 0.01:

            gear = Control.NEUTRAL

        else:

            gear = Control.FORWARD

        # =====================================================
        # PUBLISH
        # =====================================================

        self.publish_prius_control(
            steering=self.current_steering,
            throttle=self.current_throttle,
            brake=self.brake,
            gear=gear,
        )

        # =====================================================
        # DEBUG
        # =====================================================

        self.publish_debug(
            self.current_steering,
            target_speed,
        )

    # =========================================================
    # STEERING RATE LIMIT
    # =========================================================

    def rate_limit_steering(
        self,
        target,
        dt,
    ):

        max_change = (
            self.max_steering_rate
            *
            dt
        )

        difference = (
            target
            -
            self.current_steering
        )

        difference = max(
            -max_change,
            min(
                max_change,
                difference,
            ),
        )

        return (
            self.current_steering
            +
            difference
        )

    # =========================================================
    # SPEED CONTROL
    # =========================================================

    def calculate_speed(
        self,
        lateral,
        heading,
        steering,
    ):

        abs_lateral = abs(
            lateral
        )

        abs_heading = abs(
            heading
        )

        abs_steering = abs(
            steering
        )

        # Extreme turn.
        if (
            abs_heading > 0.72
            or
            abs_steering > 0.82
            or
            abs_lateral > 0.60
        ):

            return self.minimum_speed

        # Hard turn.
        if (
            abs_heading > 0.50
            or
            abs_steering > 0.62
            or
            abs_lateral > 0.45
        ):

            return self.low_speed

        # Medium turn.
        if (
            abs_heading > 0.28
            or
            abs_lateral > 0.28
            or
            abs_steering > 0.40
        ):

            return self.medium_speed

        # Straight.
        return self.max_speed

    # =========================================================
    # THROTTLE CALCULATION
    # =========================================================

    def calculate_target_throttle(
        self,
        target_speed,
        steering,
    ):

        throttle = (
            target_speed
            *
            self.max_throttle
        )

        steering_amount = abs(
            steering
        )

        turn_factor = (
            1.0
            -
            0.70
            *
            steering_amount
        )

        turn_factor = max(
            self.minimum_turn_factor,
            min(
                1.0,
                turn_factor,
            ),
        )

        throttle *= turn_factor

        if self.brake > 0.01:

            throttle = 0.0

        return max(
            0.0,
            min(
                self.max_throttle,
                throttle,
            ),
        )

    # =========================================================
    # THROTTLE RAMP
    # =========================================================

    def update_throttle(
        self,
        target,
        dt,
    ):

        difference = (
            target
            -
            self.current_throttle
        )

        if difference > 0.0:

            max_change = (
                self.throttle_rate_up
                *
                dt
            )

        else:

            max_change = (
                self.throttle_rate_down
                *
                dt
            )

        difference = max(
            -max_change,
            min(
                max_change,
                difference,
            ),
        )

        self.current_throttle += (
            difference
        )

        return max(
            0.0,
            min(
                self.max_throttle,
                self.current_throttle,
            ),
        )

    # =========================================================
    # PRIUS CONTROL PUBLISH
    # =========================================================

    def publish_prius_control(
        self,
        steering,
        throttle,
        brake,
        gear,
    ):

        msg = Control()

        msg.throttle = float(
            throttle
        )

        msg.brake = float(
            brake
        )

        msg.steer = float(
            max(
                -1.0,
                min(
                    1.0,
                    steering,
                ),
            )
        )

        msg.shift_gears = gear

        self.control_pub.publish(
            msg
        )

    # =========================================================
    # DEBUG OUTPUT
    # =========================================================

    def publish_debug(
        self,
        steering,
        speed,
    ):

        steering_msg = Float64()

        steering_msg.data = float(
            steering
        )

        self.steering_pub.publish(
            steering_msg
        )

        speed_msg = Float64()

        speed_msg.data = float(
            speed
        )

        self.speed_pub.publish(
            speed_msg
        )

    # =========================================================
    # STOP
    # =========================================================

    def stop_vehicle(self):

        msg = Control()

        msg.throttle = 0.0
        msg.brake = 1.0
        msg.steer = 0.0
        msg.shift_gears = Control.NEUTRAL

        for _ in range(10):

            self.control_pub.publish(
                msg
            )

    # =========================================================
    # DESTROY
    # =========================================================

    def destroy_node(self):

        try:

            self.stop_vehicle()

        except Exception as exc:

            self.get_logger().error(
                f"Stop command failed: {exc}"
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = VehicleController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()