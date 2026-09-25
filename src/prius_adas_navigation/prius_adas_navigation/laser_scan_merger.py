#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class LaserScanMerger(Node):

    def __init__(self):

        super().__init__('laser_scan_merger')

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.left_scan = None
        self.right_scan = None

        self.left_sub = self.create_subscription(
            LaserScan,
            '/prius/front_left_laser/scan',
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            LaserScan,
            '/prius/front_right_laser/scan',
            self.right_callback,
            10
        )

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        # ============================================================
        # OUTPUT SCAN
        # ============================================================

        self.output_angle_min = -math.pi

        self.output_angle_max = math.pi

        self.output_angle_increment = math.radians(
            1.0
        )

        self.num_bins = int(
            round(
                (
                    self.output_angle_max -
                    self.output_angle_min
                )
                /
                self.output_angle_increment
            )
        ) + 1

        # ============================================================
        # DEBUG
        # ============================================================

        self.last_finite_count = 0

        self.timer = self.create_timer(
            0.05,
            self.merge_scans
        )

        self.get_logger().info(
            'Simple Prius LiDAR merger started'
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def left_callback(self, msg):

        self.left_scan = msg

    def right_callback(self, msg):

        self.right_scan = msg

    # ================================================================
    # INSERT SCAN
    # ================================================================

    def insert_scan(
        self,
        scan,
        ranges,
        sensor_yaw
    ):

        if scan is None:

            return 0

        finite_count = 0

        for i, r in enumerate(
            scan.ranges
        ):

            if not math.isfinite(r):

                continue

            if r < scan.range_min:

                continue

            if r > scan.range_max:

                continue

            finite_count += 1

            # Original sensor ray angle
            sensor_angle = (
                scan.angle_min
                +
                i *
                scan.angle_increment
            )

            # Rotate sensor ray into base_link
            angle = (
                sensor_angle
                +
                sensor_yaw
            )

            # Normalize
            while angle > math.pi:

                angle -= 2.0 * math.pi

            while angle < -math.pi:

                angle += 2.0 * math.pi

            index = int(
                round(
                    (
                        angle -
                        self.output_angle_min
                    )
                    /
                    self.output_angle_increment
                )
            )

            if (
                index < 0
                or
                index >= len(ranges)
            ):

                continue

            # Keep the closest return
            if r < ranges[index]:

                ranges[index] = r

        return finite_count

    # ================================================================
    # MERGE
    # ================================================================

    def merge_scans(self):

        if (
            self.left_scan is None
            and
            self.right_scan is None
        ):

            return

        ranges = [
            float('inf')
            for _ in range(
                self.num_bins
            )
        ]

        # ============================================================
        # IMPORTANT
        #
        # From your verified TF:
        #
        # lasers are rotated approximately -90 degrees relative
        # to chassis.
        #
        # ============================================================

        left_count = self.insert_scan(
            self.left_scan,
            ranges,
            math.radians(-90.0)
        )

        right_count = self.insert_scan(
            self.right_scan,
            ranges,
            math.radians(-90.0)
        )

        finite_count = sum(
            1
            for value in ranges
            if math.isfinite(value)
        )

        self.last_finite_count = (
            finite_count
        )

        # ============================================================
        # DEBUG LOG
        # ============================================================

        if self.get_clock().now().nanoseconds % 500000000 < 50000000:

            self.get_logger().info(
                f'LiDAR merge: '
                f'left={left_count}, '
                f'right={right_count}, '
                f'merged={finite_count}'
            )

        # ============================================================
        # OUTPUT
        # ============================================================

        output = LaserScan()

        output.header.stamp = (
            self.get_clock().now().to_msg()
        )

        output.header.frame_id = (
            'base_link'
        )

        output.angle_min = (
            self.output_angle_min
        )

        output.angle_max = (
            self.output_angle_max
        )

        output.angle_increment = (
            self.output_angle_increment
        )

        output.time_increment = 0.0

        output.scan_time = 0.05

        output.range_min = 0.20

        output.range_max = 30.0

        output.ranges = ranges

        output.intensities = []

        self.scan_pub.publish(
            output
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = LaserScanMerger()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()