#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float64, String

from prius_msgs.msg import Control


class LaneSafetyController(Node):

    """
    Lane-following + LiDAR + four-sonar safety controller.

    NORMAL:
        Lane controller provides steering and speed.

    SLOW:
        Lane controller still controls steering,
        but safety layer reduces throttle.

    BRAKE:
        Lane steering is retained but throttle becomes zero
        and braking is applied.

    EMERGENCY:
        Full brake and zero throttle.

    IMPORTANT:
        This node is the ONLY node that should publish
        /prius/control during the integrated lane test.
    """

    def __init__(self):
        super().__init__("lane_safety_controller")

        # =========================================================
        # LANE INPUTS
        # =========================================================

        self.target_steering = 0.0
        self.target_speed = 0.0
        self.lane_brake = 0.0

        self.last_steering_time = 0.0
        self.last_speed_time = 0.0
        self.last_brake_time = 0.0

        self.lane_timeout = 0.50

        self.steering_sub = self.create_subscription(
            Float64,
            "/adas/target_steering",
            self.steering_callback,
            10,
        )

        self.speed_sub = self.create_subscription(
            Float64,
            "/adas/target_speed",
            self.speed_callback,
            10,
        )

        self.brake_sub = self.create_subscription(
            Float64,
            "/adas/brake",
            self.brake_callback,
            10,
        )

        # =========================================================
        # LIDAR
        # =========================================================

        self.scan: Optional[LaserScan] = None
        self.last_scan_time = 0.0

        self.scan_timeout = 0.40

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/prius/scan",
            self.scan_callback,
            10,
        )

        # =========================================================
        # FOUR FRONT SONARS
        # =========================================================

        self.left_far: Optional[Range] = None
        self.left_middle: Optional[Range] = None
        self.right_far: Optional[Range] = None
        self.right_middle: Optional[Range] = None

        self.left_far_time = 0.0
        self.left_middle_time = 0.0
        self.right_far_time = 0.0
        self.right_middle_time = 0.0

        self.sonar_timeout = 0.60

        self.left_far_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/left_far_range",
            self.left_far_callback,
            10,
        )

        self.left_middle_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/left_middle_range",
            self.left_middle_callback,
            10,
        )

        self.right_far_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/right_far_range",
            self.right_far_callback,
            10,
        )

        self.right_middle_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/right_middle_range",
            self.right_middle_callback,
            10,
        )

        # =========================================================
        # ODOM
        # =========================================================

        self.speed = 0.0
        self.last_odom_time = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # =========================================================
        # VEHICLE PARAMETERS
        # =========================================================

        # Same general lane-controller scaling you were using,
        # but safety can reduce it.
        self.speed_scale = 3.0
        self.max_throttle = 0.09

        self.minimum_turn_factor = 0.40

        # =========================================================
        # SAFETY DISTANCES
        # =========================================================

        # These are desired clearance thresholds rather than
        # hard vehicle dimensions.

        self.front_warning = 2.50
        self.front_brake = 1.20
        self.front_emergency = 0.75

        self.side_warning = 1.20
        self.side_brake = 0.70
        self.side_emergency = 0.35

        self.rear_warning = 1.50
        self.rear_emergency = 0.55

        # Four sonar close-range protection.
        self.sonar_warning = 0.85
        self.sonar_brake = 0.55
        self.sonar_emergency = 0.35

        # =========================================================
        # TTC
        # =========================================================

        # Time-to-collision limits.
        self.warning_ttc = 2.5
        self.braking_ttc = 1.5
        self.emergency_ttc = 0.80

        # =========================================================
        # SAFETY THROTTLES
        # =========================================================

        self.slow_throttle = 0.004
        self.warning_throttle = 0.008

        self.brake_command = 0.45
        self.emergency_brake = 1.0

        # =========================================================
        # OUTPUT
        # =========================================================

        self.control_pub = self.create_publisher(
            Control,
            "/prius/control",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/adas/safety_state",
            10,
        )

        # =========================================================
        # LOOP
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        self.last_status_time = time.monotonic()

        self.get_logger().info(
            "================================================"
        )

        self.get_logger().info(
            "LANE + LiDAR + SONAR safety controller started"
        )

        self.get_logger().info(
            "Lane controller = primary steering"
        )

        self.get_logger().info(
            "LiDAR + sonar = safety override"
        )

        self.get_logger().info(
            "================================================"
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def steering_callback(self, msg):
        self.target_steering = max(
            -1.0,
            min(
                1.0,
                float(msg.data),
            ),
        )

        self.last_steering_time = time.monotonic()

    def speed_callback(self, msg):
        self.target_speed = max(
            0.0,
            min(
                1.0,
                float(msg.data),
            ),
        )

        self.last_speed_time = time.monotonic()

    def brake_callback(self, msg):
        self.lane_brake = max(
            0.0,
            min(
                1.0,
                float(msg.data),
            ),
        )

        self.last_brake_time = time.monotonic()

    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_time = time.monotonic()

    def left_far_callback(self, msg):
        self.left_far = msg
        self.left_far_time = time.monotonic()

    def left_middle_callback(self, msg):
        self.left_middle = msg
        self.left_middle_time = time.monotonic()

    def right_far_callback(self, msg):
        self.right_far = msg
        self.right_far_time = time.monotonic()

    def right_middle_callback(self, msg):
        self.right_middle_time = time.monotonic()
        self.right_middle = msg

    def odom_callback(self, msg):

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.speed = math.hypot(
            vx,
            vy,
        )

        self.last_odom_time = time.monotonic()

    # =========================================================
    # TIMEOUTS
    # =========================================================

    def sensor_age(self, timestamp):
        if timestamp <= 0.0:
            return float("inf")

        return time.monotonic() - timestamp

    def lane_valid(self):

        return (
            self.sensor_age(
                self.last_steering_time
            ) <= self.lane_timeout
            and
            self.sensor_age(
                self.last_speed_time
            ) <= self.lane_timeout
        )

    def lidar_valid(self):

        return (
            self.scan is not None
            and
            self.sensor_age(
                self.last_scan_time
            ) <= self.scan_timeout
        )

    # =========================================================
    # RANGE CLEANING
    # =========================================================

    def clean_lidar(self, value):

        if not math.isfinite(value):
            return 30.0

        if value <= 0.0:
            return 30.0

        return min(
            max(value, 0.12),
            30.0,
        )

    def clean_sonar(self, msg):

        if msg is None:
            return float("inf")

        if not math.isfinite(msg.range):
            return float("inf")

        return max(
            msg.min_range,
            min(
                msg.range,
                msg.max_range,
            ),
        )

    # =========================================================
    # LIDAR ANGLE
    # =========================================================

    def angle_index(self, angle_deg):

        if not self.lidar_valid():
            return None

        angle_rad = math.radians(
            angle_deg
        )

        index = int(
            round(
                (
                    angle_rad
                    - self.scan.angle_min
                )
                /
                self.scan.angle_increment
            )
        )

        index = max(
            0,
            min(
                len(self.scan.ranges) - 1,
                index,
            ),
        )

        return index

    # =========================================================
    # LIDAR SECTOR
    # =========================================================

    def sector_min(
        self,
        start_deg,
        end_deg,
    ):

        if not self.lidar_valid():
            return float("inf")

        start = self.angle_index(
            start_deg
        )

        end = self.angle_index(
            end_deg
        )

        if start is None or end is None:
            return float("inf")

        if start > end:
            start, end = end, start

        values = []

        for i in range(
            start,
            end + 1,
        ):

            values.append(
                self.clean_lidar(
                    self.scan.ranges[i]
                )
            )

        if not values:
            return float("inf")

        return min(values)

    # =========================================================
    # 360 DEGREE SAFETY
    # =========================================================

    def lidar_clearances(self):

        return {
            "front": self.sector_min(
                -18.0,
                18.0,
            ),

            "front_left": self.sector_min(
                18.0,
                55.0,
            ),

            "left": self.sector_min(
                55.0,
                120.0,
            ),

            "rear_left": self.sector_min(
                120.0,
                170.0,
            ),

            "rear": min(
                self.sector_min(
                    170.0,
                    180.0,
                ),
                self.sector_min(
                    -180.0,
                    -170.0,
                ),
            ),

            "rear_right": self.sector_min(
                -170.0,
                -120.0,
            ),

            "right": self.sector_min(
                -120.0,
                -55.0,
            ),

            "front_right": self.sector_min(
                -55.0,
                -18.0,
            ),
        }

    # =========================================================
    # SONAR
    # =========================================================

    def sonar_clearances(self):

        left = min(
            self.clean_sonar(
                self.left_far
            ),
            self.clean_sonar(
                self.left_middle
            ),
        )

        right = min(
            self.clean_sonar(
                self.right_far
            ),
            self.clean_sonar(
                self.right_middle
            ),
        )

        return {
            "left": left,
            "right": right,
        }

    # =========================================================
    # TTC
    # =========================================================

    def calculate_ttc(
        self,
        distance,
    ):

        if self.speed <= 0.02:
            return float("inf")

        return distance / self.speed

    # =========================================================
    # SAFETY STATE
    # =========================================================

    def determine_safety(
        self,
        lidar,
        sonar,
    ):

        front = lidar["front"]
        rear = lidar["rear"]
        left = lidar["left"]
        right = lidar["right"]

        sonar_left = sonar["left"]
        sonar_right = sonar["right"]

        # -----------------------------------------------------
        # LiDAR hard emergency
        # -----------------------------------------------------

        if front <= self.front_emergency:
            return "EMERGENCY"

        if rear <= self.rear_emergency:
            return "EMERGENCY"

        # Only use side hard-stop at very close range.
        if left <= self.side_emergency:
            return "EMERGENCY"

        if right <= self.side_emergency:
            return "EMERGENCY"

        # -----------------------------------------------------
        # Sonar hard emergency
        # -----------------------------------------------------

        if sonar_left <= self.sonar_emergency:
            return "EMERGENCY"

        if sonar_right <= self.sonar_emergency:
            return "EMERGENCY"

        # -----------------------------------------------------
        # TTC
        # -----------------------------------------------------

        front_ttc = self.calculate_ttc(
            front
        )

        if (
            self.speed > 0.02
            and front_ttc
            <= self.emergency_ttc
        ):
            return "EMERGENCY"

        if (
            self.speed > 0.02
            and front_ttc
            <= self.braking_ttc
        ):
            return "BRAKE"

        # -----------------------------------------------------
        # Front warning / brake
        # -----------------------------------------------------

        if front <= self.front_brake:
            return "BRAKE"

        if (
            sonar_left <= self.sonar_brake
            and
            sonar_right <= self.sonar_brake
        ):
            return "BRAKE"

        # -----------------------------------------------------
        # Side / rear slow
        # -----------------------------------------------------

        if front <= self.front_warning:
            return "SLOW"

        if rear <= self.rear_warning:
            return "SLOW"

        if left <= self.side_warning:
            return "SLOW"

        if right <= self.side_warning:
            return "SLOW"

        if sonar_left <= self.sonar_warning:
            return "SLOW"

        if sonar_right <= self.sonar_warning:
            return "SLOW"

        return "CLEAR"

    # =========================================================
    # SAFE SPEED
    # =========================================================

    def calculate_lane_throttle(
        self,
    ):

        throttle = (
            self.target_speed
            * self.speed_scale
        )

        steering_amount = abs(
            self.target_steering
        )

        turn_factor = (
            1.0
            - 0.60 * steering_amount
        )

        turn_factor = max(
            self.minimum_turn_factor,
            min(
                1.0,
                turn_factor,
            ),
        )

        throttle *= turn_factor

        return max(
            0.0,
            min(
                self.max_throttle,
                throttle,
            ),
        )

    # =========================================================
    # FINAL CONTROL
    # =========================================================

    def publish_control(
        self,
        throttle,
        brake,
        steering,
    ):

        msg = Control()

        msg.throttle = float(
            throttle
        )

        msg.brake = float(
            brake
        )

        msg.steer = float(
            steering
        )

        msg.shift_gears = (
            Control.FORWARD
        )

        self.control_pub.publish(
            msg
        )

    # =========================================================
    # STATUS
    # =========================================================

    def publish_state(
        self,
        state,
    ):

        msg = String()

        msg.data = state

        self.status_pub.publish(
            msg
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        # ======================================================
        # LANE COMMAND LOST
        # ======================================================

        if not self.lane_valid():

            self.publish_control(
                throttle=0.0,
                brake=1.0,
                steering=0.0,
            )

            self.publish_state(
                "LANE_COMMAND_TIMEOUT"
            )

            return

        # ======================================================
        # LiDAR LOST
        # ======================================================

        if not self.lidar_valid():

            self.publish_control(
                throttle=0.0,
                brake=1.0,
                steering=0.0,
            )

            self.publish_state(
                "LIDAR_TIMEOUT"
            )

            return

        # ======================================================
        # SENSOR DATA
        # ======================================================

        lidar = self.lidar_clearances()

        sonar = self.sonar_clearances()

        # ======================================================
        # SAFETY
        # ======================================================

        state = self.determine_safety(
            lidar,
            sonar,
        )

        # ======================================================
        # LANE BRAKE
        # ======================================================

        if self.lane_brake > 0.01:

            self.publish_control(
                throttle=0.0,
                brake=self.lane_brake,
                steering=0.0,
            )

            self.publish_state(
                "LANE_BRAKE"
            )

            return

        # ======================================================
        # EMERGENCY
        # ======================================================

        if state == "EMERGENCY":

            self.publish_control(
                throttle=0.0,
                brake=self.emergency_brake,
                steering=0.0,
            )

            self.publish_state(
                "EMERGENCY"
            )

            return

        # ======================================================
        # LANE STEERING
        # ======================================================

        steering = self.target_steering

        throttle = self.calculate_lane_throttle()

        brake = 0.0

        # ======================================================
        # SAFETY SPEED REDUCTION
        # ======================================================

        if state == "SLOW":

            throttle = min(
                throttle,
                self.slow_throttle,
            )

            brake = 0.0

        elif state == "BRAKE":

            throttle = 0.0

            brake = self.brake_command

        # ======================================================
        # PUBLISH
        # ======================================================

        self.publish_control(
            throttle=throttle,
            brake=brake,
            steering=steering,
        )

        self.publish_state(
            state
        )

        # ======================================================
        # DIAGNOSTIC LOG
        # ======================================================

        now = time.monotonic()

        if now - self.last_status_time >= 1.0:

            self.last_status_time = now

            self.get_logger().info(
                "LANE+SAFETY | "
                f"state={state} "
                f"speed={self.speed:.2f} "
                f"front={lidar['front']:.2f}m "
                f"left={lidar['left']:.2f}m "
                f"right={lidar['right']:.2f}m "
                f"rear={lidar['rear']:.2f}m "
                f"sonarL={sonar['left']:.2f}m "
                f"sonarR={sonar['right']:.2f}m "
                f"steer={steering:.2f} "
                f"thr={throttle:.3f}"
            )

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def destroy_node(self):

        try:
            self.publish_control(
                throttle=0.0,
                brake=1.0,
                steering=0.0,
            )
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = LaneSafetyController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
