#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range

from prius_msgs.msg import Control


class LidarSonarSafety(Node):
    """
    360-degree LiDAR + 4-sonar safety controller.

    Current standalone operation:
        LiDAR + sonar -> safe steering + speed + braking
        output       -> /prius/control

    Later lane integration:
        lane controller -> desired command
                       -> this safety layer
                       -> /prius/control

    The safety layer always has authority to reduce speed or brake.
    """

    def __init__(self):
        super().__init__("lidar_sonar_safety")

        # =========================================================
        # VEHICLE GEOMETRY
        # =========================================================

        # Conservative dimensions for the simulated Prius.
        self.vehicle_length_front = 3.0
        self.vehicle_length_rear = 1.5

        self.vehicle_half_width = 0.95

        # Extra clearance around the complete vehicle.
        self.safety_margin = 0.50

        # Resulting protected envelope.
        self.front_safe_distance = (
            self.vehicle_length_front
            + self.safety_margin
        )

        self.rear_safe_distance = (
            self.vehicle_length_rear
            + self.safety_margin
        )

        self.side_safe_distance = (
            self.vehicle_half_width
            + self.safety_margin
        )

        # =========================================================
        # SPEED
        # =========================================================

        self.max_speed = 0.12
        self.cruise_speed = 0.08
        self.slow_speed = 0.035

        # Current commanded throttle values.
        self.max_throttle = 0.012
        self.cruise_throttle = 0.008
        self.slow_throttle = 0.004

        # =========================================================
        # BRAKING / TTC
        # =========================================================

        # Begin braking when predicted collision is close.
        self.warning_ttc = 2.5
        self.braking_ttc = 1.5
        self.emergency_ttc = 0.8

        self.maximum_brake = 1.0
        self.normal_brake = 0.25

        # =========================================================
        # STATIC CLEARANCES
        # =========================================================

        # LiDAR-based hard stop distances.
        self.front_hard_stop = 0.90
        self.rear_hard_stop = 0.70
        self.side_hard_stop = 0.55

        # LiDAR warning distances.
        self.front_warning = 2.5
        self.rear_warning = 1.8
        self.side_warning = 1.5

        # Sonar thresholds.
        self.sonar_hard_stop = 0.45
        self.sonar_warning = 0.85

        # =========================================================
        # LiDAR
        # =========================================================

        self.max_lidar_range = 30.0

        self.scan: Optional[LaserScan] = None
        self.last_scan_time = None
        self.scan_timeout = 0.40

        # =========================================================
        # SONAR
        # =========================================================

        self.left_far: Optional[Range] = None
        self.left_middle: Optional[Range] = None
        self.right_far: Optional[Range] = None
        self.right_middle: Optional[Range] = None

        self.last_left_far_time = None
        self.last_left_middle_time = None
        self.last_right_far_time = None
        self.last_right_middle_time = None

        self.sonar_timeout = 0.50

        # =========================================================
        # ODOMETRY
        # =========================================================

        self.speed = 0.0
        self.last_odom_time = None
        self.odom_timeout = 0.50

        # =========================================================
        # CONTROL
        # =========================================================

        self.steering_sign = -1.0

        self.last_steering = 0.0

        self.max_steering = 1.0
        self.max_steering_step = 0.05

        # =========================================================
        # ROS SUBSCRIBERS
        # =========================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/prius/scan",
            self.scan_callback,
            10,
        )

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

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # =========================================================
        # OUTPUT
        # =========================================================

        self.control_pub = self.create_publisher(
            Control,
            "/prius/control",
            10,
        )

        # Diagnostic output.
        self.status_pub = self.create_publisher(
            Twist,
            "/lidar_sonar_safety/status",
            10,
        )

        # =========================================================
        # CONTROL TIMER
        # =========================================================

        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.control_loop,
        )

        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            "LiDAR + SONAR 360 safety controller started."
        )

        self.get_logger().info(
            "Safety layer monitors front/rear/left/right."
        )

    # =========================================================
    # TIME
    # =========================================================

    def now(self) -> float:
        return (
            self.get_clock().now().nanoseconds
            * 1e-9
        )

    def age(self, timestamp) -> float:
        if timestamp is None:
            return float("inf")

        return max(
            0.0,
            self.now() - timestamp,
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        self.scan = msg
        self.last_scan_time = self.now()

    def left_far_callback(self, msg: Range):
        self.left_far = msg
        self.last_left_far_time = self.now()

    def left_middle_callback(self, msg: Range):
        self.left_middle = msg
        self.last_left_middle_time = self.now()

    def right_far_callback(self, msg: Range):
        self.right_far = msg
        self.last_right_far_time = self.now()

    def right_middle_callback(self, msg: Range):
        self.right_middle = msg
        self.last_right_middle_time = self.now()

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.speed = math.hypot(
            vx,
            vy,
        )

        self.last_odom_time = self.now()

    # =========================================================
    # SENSOR VALIDITY
    # =========================================================

    def lidar_valid(self) -> bool:
        return (
            self.scan is not None
            and self.age(self.last_scan_time)
            <= self.scan_timeout
            and len(self.scan.ranges) > 0
        )

    def odom_valid(self) -> bool:
        return (
            self.age(self.last_odom_time)
            <= self.odom_timeout
        )

    # =========================================================
    # RANGE CLEANING
    # =========================================================

    def clean_lidar(self, value: float) -> float:

        if not math.isfinite(value):
            return self.max_lidar_range

        if value <= 0.0:
            return self.max_lidar_range

        return min(
            max(value, 0.12),
            self.max_lidar_range,
        )

    def clean_sonar(
        self,
        msg: Optional[Range],
    ) -> float:

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
    # LiDAR ANGLE INDEX
    # =========================================================

    def angle_index(
        self,
        angle_deg: float,
    ) -> Optional[int]:

        if not self.lidar_valid():
            return None

        angle_rad = math.radians(angle_deg)

        index = int(
            round(
                (angle_rad - self.scan.angle_min)
                / self.scan.angle_increment
            )
        )

        if index < 0:
            index = 0

        if index >= len(self.scan.ranges):
            index = len(self.scan.ranges) - 1

        return index

    # =========================================================
    # LiDAR SECTOR
    # =========================================================

    def sector_min(
        self,
        start_deg: float,
        end_deg: float,
    ) -> float:

        if not self.lidar_valid():
            return float("inf")

        a = self.angle_index(start_deg)
        b = self.angle_index(end_deg)

        if a is None or b is None:
            return float("inf")

        if a > b:
            a, b = b, a

        values = []

        for i in range(a, b + 1):

            values.append(
                self.clean_lidar(
                    self.scan.ranges[i]
                )
            )

        if not values:
            return float("inf")

        return min(values)

    # =========================================================
    # 360 DEGREE CLEARANCE
    # =========================================================

    def get_clearances(self):

        front = self.sector_min(
            -18.0,
            18.0,
        )

        front_left = self.sector_min(
            18.0,
            55.0,
        )

        left = self.sector_min(
            55.0,
            125.0,
        )

        rear_left = self.sector_min(
            125.0,
            170.0,
        )

        rear = min(
            self.sector_min(170.0, 180.0),
            self.sector_min(-180.0, -170.0),
        )

        rear_right = self.sector_min(
            -170.0,
            -125.0,
        )

        right = self.sector_min(
            -125.0,
            -55.0,
        )

        front_right = self.sector_min(
            -55.0,
            -18.0,
        )

        return {
            "front": front,
            "front_left": front_left,
            "left": left,
            "rear_left": rear_left,
            "rear": rear,
            "rear_right": rear_right,
            "right": right,
            "front_right": front_right,
        }

    # =========================================================
    # SONAR VALUES
    # =========================================================

    def sonar_values(self):

        return {
            "left_far": self.clean_sonar(
                self.left_far
            ),
            "left_middle": self.clean_sonar(
                self.left_middle
            ),
            "right_far": self.clean_sonar(
                self.right_far
            ),
            "right_middle": self.clean_sonar(
                self.right_middle
            ),
        }

    # =========================================================
    # SONAR SIDE MINIMA
    # =========================================================

    def sonar_left(self, sonar):

        return min(
            sonar["left_far"],
            sonar["left_middle"],
        )

    def sonar_right(self, sonar):

        return min(
            sonar["right_far"],
            sonar["right_middle"],
        )

    # =========================================================
    # APPROXIMATE TTC
    # =========================================================

    def ttc(
        self,
        distance: float,
    ) -> float:

        if self.speed <= 0.01:
            return float("inf")

        return distance / self.speed

    # =========================================================
    # SAFETY CLASSIFICATION
    # =========================================================

    def classify(
        self,
        clearance,
        sonar,
    ):

        front = clearance["front"]
        rear = clearance["rear"]
        left = clearance["left"]
        right = clearance["right"]

        sonar_left = self.sonar_left(sonar)
        sonar_right = self.sonar_right(sonar)

        # ------------------------------------------------------
        # LiDAR hard limits
        # ------------------------------------------------------

        if front <= self.front_hard_stop:
            return "EMERGENCY"

        if rear <= self.rear_hard_stop:
            return "EMERGENCY"

        if left <= self.side_hard_stop:
            return "EMERGENCY"

        if right <= self.side_hard_stop:
            return "EMERGENCY"

        # ------------------------------------------------------
        # Sonar hard limits
        # ------------------------------------------------------

        if sonar_left <= self.sonar_hard_stop:
            return "EMERGENCY"

        if sonar_right <= self.sonar_hard_stop:
            return "EMERGENCY"

        # ------------------------------------------------------
        # TTC protection
        # ------------------------------------------------------

        front_ttc = self.ttc(front)

        if (
            self.speed > 0.03
            and front_ttc <= self.emergency_ttc
        ):
            return "EMERGENCY"

        if (
            self.speed > 0.03
            and front_ttc <= self.braking_ttc
        ):
            return "BRAKE"

        # ------------------------------------------------------
        # Warning distance
        # ------------------------------------------------------

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
    # CHOOSE SAFE STEERING
    # =========================================================

    def choose_steering(
        self,
        clearance,
        sonar,
    ):

        left_space = min(
            clearance["front_left"],
            clearance["left"],
            clearance["rear_left"],
            self.sonar_left(sonar),
        )

        right_space = min(
            clearance["front_right"],
            clearance["right"],
            clearance["rear_right"],
            self.sonar_right(sonar),
        )

        front = clearance["front"]

        # ------------------------------------------------------
        # Clear ahead
        # ------------------------------------------------------

        if (
            front > self.front_warning
            and left_space > self.side_warning
            and right_space > self.side_warning
        ):
            return 0.0

        # ------------------------------------------------------
        # Obstacle ahead
        # ------------------------------------------------------

        if left_space > right_space:
            target = -0.65
        elif right_space > left_space:
            target = 0.65
        else:
            target = 0.0

        return target

    # =========================================================
    # STEERING RATE LIMIT
    # =========================================================

    def limit_steering(
        self,
        target,
    ):

        delta = (
            target
            - self.last_steering
        )

        delta = max(
            -self.max_steering_step,
            min(
                self.max_steering_step,
                delta,
            ),
        )

        self.last_steering += delta

        return self.last_steering

    # =========================================================
    # BRAKE COMMAND
    # =========================================================

    def brake_value(
        self,
        front_distance,
        state,
    ):

        if state == "EMERGENCY":
            return 1.0

        if state == "BRAKE":
            return 0.70

        if state == "SLOW":
            return 0.20

        return 0.0

    # =========================================================
    # THROTTLE
    # =========================================================

    def throttle_value(
        self,
        state,
        steering,
    ):

        if state in (
            "EMERGENCY",
            "BRAKE",
        ):
            return 0.0

        if state == "SLOW":
            return self.slow_throttle

        if abs(steering) > 0.65:
            return self.slow_throttle

        return self.cruise_throttle

    # =========================================================
    # PUBLISH
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

        msg.shift_gears = Control.FORWARD

        self.control_pub.publish(
            msg
        )

    # =========================================================
    # STATUS OUTPUT
    # =========================================================

    def publish_status(
        self,
        clearance,
        sonar,
        state,
    ):

        msg = Twist()

        msg.linear.x = self.speed

        msg.linear.y = clearance["front"]
        msg.linear.z = clearance["rear"]

        msg.angular.x = clearance["left"]
        msg.angular.y = clearance["right"]

        # z = encoded minimum sonar distance.
        msg.angular.z = min(
            self.sonar_left(sonar),
            self.sonar_right(sonar),
        )

        self.status_pub.publish(msg)

    # =========================================================
    # LOGGING
    # =========================================================

    def periodic_log(
        self,
        clearance,
        sonar,
        state,
        steering,
    ):

        now = self.get_clock().now()

        if (
            now - self.last_log_time
        ).nanoseconds < 1_000_000_000:
            return

        self.last_log_time = now

        self.get_logger().info(
            "SAFETY | "
            f"state={state} "
            f"speed={self.speed:.2f} "
            f"F={clearance['front']:.2f} "
            f"R={clearance['rear']:.2f} "
            f"L={clearance['left']:.2f} "
            f"Right={clearance['right']:.2f} "
            f"S-L={self.sonar_left(sonar):.2f} "
            f"S-R={self.sonar_right(sonar):.2f} "
            f"steer={steering:.2f}"
        )

    # =========================================================
    # MAIN LOOP
    # =========================================================

    def control_loop(self):

        # ======================================================
        # LiDAR timeout -> fail safe
        # ======================================================

        if not self.lidar_valid():

            self.publish_control(
                throttle=0.0,
                brake=1.0,
                steering=0.0,
            )

            return

        # ======================================================
        # GET SENSOR DATA
        # ======================================================

        clearance = self.get_clearances()
        sonar = self.sonar_values()

        # ======================================================
        # CLASSIFY
        # ======================================================

        state = self.classify(
            clearance,
            sonar,
        )

        # ======================================================
        # EMERGENCY
        # ======================================================

        if state == "EMERGENCY":

            self.last_steering = 0.0

            self.publish_control(
                throttle=0.0,
                brake=1.0,
                steering=0.0,
            )

            self.publish_status(
                clearance,
                sonar,
                state,
            )

            self.periodic_log(
                clearance,
                sonar,
                state,
                0.0,
            )

            return

        # ======================================================
        # STEERING
        # ======================================================

        target_steering = (
            self.choose_steering(
                clearance,
                sonar,
            )
        )

        steering = self.limit_steering(
            target_steering
        )

        physical_steering = (
            steering
            * self.steering_sign
        )

        # ======================================================
        # SPEED
        # ======================================================

        throttle = self.throttle_value(
            state,
            steering,
        )

        brake = self.brake_value(
            clearance["front"],
            state,
        )

        # ======================================================
        # PUBLISH
        # ======================================================

        self.publish_control(
            throttle=throttle,
            brake=brake,
            steering=physical_steering,
        )

        self.publish_status(
            clearance,
            sonar,
            state,
        )

        self.periodic_log(
            clearance,
            sonar,
            state,
            physical_steering,
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

    node = LidarSonarSafety()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()