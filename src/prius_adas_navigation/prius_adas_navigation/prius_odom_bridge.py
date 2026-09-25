#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class PriusOdomBridge(Node):

    def __init__(self):
        super().__init__('prius_odom_bridge')

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter(
            'ground_truth_topic',
            '/prius/ground_truth'
        )

        self.declare_parameter(
            'odom_topic',
            '/odom'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        self.declare_parameter(
            'base_frame',
            'base_link'
        )

        # ============================================================
        # Ground-truth subscriber
        #
        # /prius/ground_truth:
        #     nav_msgs/Odometry
        #
        # Pose is the Prius chassis pose in world coordinates.
        # Twist is expressed in the child frame of that odometry,
        # which is the chassis frame in this simulation.
        # ============================================================

        self.ground_truth_topic = self.get_parameter(
            'ground_truth_topic'
        ).value

        self.gt_sub = self.create_subscription(
            Odometry,
            self.ground_truth_topic,
            self.odom_callback,
            20
        )

        # ============================================================
        # /odom publisher
        # ============================================================

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            20
        )

        # ============================================================
        # TF broadcaster
        #
        # Publishes:
        #
        #     odom -> base_link
        # ============================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        # ============================================================
        # Known fixed transform:
        #
        #     base_link -> chassis
        #
        # x   = +1.45 m
        # y   =  0.00 m
        # yaw = +90 deg
        #
        # Therefore:
        #
        #     chassis -> base_link
        #
        # yaw = -90 deg
        # ============================================================

        self.base_to_chassis_x = 1.45
        self.base_to_chassis_y = 0.0
        self.base_to_chassis_yaw = math.pi / 2.0

        self.chassis_to_base_yaw = (
            -self.base_to_chassis_yaw
        )

        # ============================================================
        # Initial odom origin
        # ============================================================

        self.initialized = False

        self.initial_base_x = 0.0
        self.initial_base_y = 0.0
        self.initial_base_yaw = 0.0

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'Prius odometry bridge started'
        )

        self.get_logger().info(
            f'Ground truth : {self.ground_truth_topic}'
        )

        self.get_logger().info(
            f'Odometry     : {self.odom_topic}'
        )

        self.get_logger().info(
            f'Frames       : {self.odom_frame} -> {self.base_frame}'
        )

        self.get_logger().info(
            'base_link -> chassis: '
            'x=1.45 m, y=0.0 m, yaw=+90 deg'
        )

        self.get_logger().info(
            'chassis -> base_link twist rotation: -90 deg'
        )

        self.get_logger().info(
            '================================================'
        )

    # ================================================================
    # Quaternion -> yaw
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = (
            2.0
            * (
                q.w * q.z
                + q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            - 2.0
            * (
                q.y * q.y
                + q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ================================================================
    # Yaw -> quaternion
    # ================================================================

    @staticmethod
    def yaw_to_quaternion(yaw):

        qz = math.sin(
            yaw / 2.0
        )

        qw = math.cos(
            yaw / 2.0
        )

        return qz, qw

    # ================================================================
    # Rotate a 2D vector
    # ================================================================

    @staticmethod
    def rotate_vector(
        x,
        y,
        yaw
    ):

        c = math.cos(yaw)
        s = math.sin(yaw)

        return (
            c * x - s * y,
            s * x + c * y
        )

    # ================================================================
    # Normalize angle
    # ================================================================

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # ================================================================
    # Convert chassis pose to base_link pose
    #
    # Known:
    #
    #   T_world_chassis
    #
    # and:
    #
    #   T_base_chassis
    #
    # We want:
    #
    #   T_world_base
    #
    # Since:
    #
    #   p_chassis =
    #       p_base +
    #       R(base_world) * p_base_chassis
    #
    # therefore:
    #
    #   p_base =
    #       p_chassis -
    #       R(base_world) * p_base_chassis
    # ================================================================

    def chassis_pose_to_base_pose(
        self,
        chassis_x,
        chassis_y,
        chassis_yaw
    ):

        # ------------------------------------------------------------
        # base_link world yaw
        #
        # base -> chassis = +90 deg
        #
        # therefore:
        #
        # chassis -> base = -90 deg
        # ------------------------------------------------------------

        base_yaw_world = self.normalize_angle(
            chassis_yaw
            - self.base_to_chassis_yaw
        )

        # ------------------------------------------------------------
        # Position of chassis relative to base, expressed in base
        # coordinates.
        # ------------------------------------------------------------

        offset_x_world, offset_y_world = (
            self.rotate_vector(
                self.base_to_chassis_x,
                self.base_to_chassis_y,
                base_yaw_world
            )
        )

        # ------------------------------------------------------------
        # Solve for base position.
        # ------------------------------------------------------------

        base_x_world = (
            chassis_x
            - offset_x_world
        )

        base_y_world = (
            chassis_y
            - offset_y_world
        )

        return (
            base_x_world,
            base_y_world,
            base_yaw_world
        )

    # ================================================================
    # Main callback
    # ================================================================

    def odom_callback(self, msg):

        # ============================================================
        # 1. Read chassis pose from ground truth
        # ============================================================

        chassis_x = float(
            msg.pose.pose.position.x
        )

        chassis_y = float(
            msg.pose.pose.position.y
        )

        chassis_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # ============================================================
        # 2. Convert chassis pose -> base_link pose
        # ============================================================

        (
            base_x_world,
            base_y_world,
            base_yaw_world
        ) = self.chassis_pose_to_base_pose(
            chassis_x,
            chassis_y,
            chassis_yaw
        )

        # ============================================================
        # 3. Initialize local odom frame using BASE LINK pose
        # ============================================================

        if not self.initialized:

            self.initial_base_x = (
                base_x_world
            )

            self.initial_base_y = (
                base_y_world
            )

            self.initial_base_yaw = (
                base_yaw_world
            )

            self.initialized = True

            self.get_logger().info(
                'Initial BASE LINK pose: '
                f'x={base_x_world:.3f}, '
                f'y={base_y_world:.3f}, '
                f'yaw={math.degrees(base_yaw_world):.2f} deg'
            )

        # ============================================================
        # 4. Position relative to initial odom origin
        # ============================================================

        dx_world = (
            base_x_world
            - self.initial_base_x
        )

        dy_world = (
            base_y_world
            - self.initial_base_y
        )

        # Rotate world displacement into initial odom frame.

        cos0 = math.cos(
            -self.initial_base_yaw
        )

        sin0 = math.sin(
            -self.initial_base_yaw
        )

        odom_x = (
            cos0 * dx_world
            - sin0 * dy_world
        )

        odom_y = (
            sin0 * dx_world
            + cos0 * dy_world
        )

        # ============================================================
        # 5. Relative orientation
        # ============================================================

        odom_yaw = self.normalize_angle(
            base_yaw_world
            - self.initial_base_yaw
        )

        qz, qw = self.yaw_to_quaternion(
            odom_yaw
        )

        # ============================================================
        # 6. Velocity conversion
        #
        # THIS IS THE IMPORTANT FIX.
        #
        # /prius/ground_truth is nav_msgs/Odometry.
        #
        # Its twist is expressed in the child frame.
        # In your simulation the child is the chassis frame.
        #
        # Static relationship:
        #
        #     base_link -> chassis = +90 deg
        #
        # Therefore:
        #
        #     chassis -> base_link = -90 deg
        #
        # We transform the CHASSIS-FRAME velocity directly
        # into BASE-LINK coordinates.
        #
        # This is NOT a world->base rotation.
        # ============================================================

        vx_chassis = float(
            msg.twist.twist.linear.x
        )

        vy_chassis = float(
            msg.twist.twist.linear.y
        )

        vz_chassis = float(
            msg.twist.twist.linear.z
        )

        # ------------------------------------------------------------
        # chassis -> base_link
        # fixed rotation = -90 degrees
        #
        # [vx_base]   [ 0  1] [vx_chassis]
        # [vy_base] = [-1  0] [vy_chassis]
        # ------------------------------------------------------------

        (
            vx_base,
            vy_base
        ) = self.rotate_vector(
            vx_chassis,
            vy_chassis,
            self.chassis_to_base_yaw
        )

        # Angular velocity around Z is unchanged by a planar
        # rotation around the same Z axis.

        wz = float(
            msg.twist.twist.angular.z
        )

        # ============================================================
        # 7. Create Odometry message
        # ============================================================

        odom = Odometry()

        odom.header.stamp = (
            msg.header.stamp
        )

        odom.header.frame_id = (
            self.odom_frame
        )

        odom.child_frame_id = (
            self.base_frame
        )

        # ------------------------------------------------------------
        # Pose
        # ------------------------------------------------------------

        odom.pose.pose.position.x = (
            odom_x
        )

        odom.pose.pose.position.y = (
            odom_y
        )

        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # ------------------------------------------------------------
        # Velocity in base_link frame
        # ------------------------------------------------------------

        odom.twist.twist.linear.x = (
            vx_base
        )

        odom.twist.twist.linear.y = (
            vy_base
        )

        odom.twist.twist.linear.z = (
            vz_chassis
        )

        odom.twist.twist.angular.x = float(
            msg.twist.twist.angular.x
        )

        odom.twist.twist.angular.y = float(
            msg.twist.twist.angular.y
        )

        odom.twist.twist.angular.z = (
            wz
        )

        # ============================================================
        # 8. Covariance
        # ============================================================

        odom.pose.covariance[0] = 0.001

        odom.pose.covariance[7] = 0.001

        odom.pose.covariance[35] = 0.01

        odom.twist.covariance[0] = 0.001

        odom.twist.covariance[7] = 0.001

        odom.twist.covariance[35] = 0.01

        # ============================================================
        # 9. Publish /odom
        # ============================================================

        self.odom_pub.publish(
            odom
        )

        # ============================================================
        # 10. Publish odom -> base_link TF
        # ============================================================

        transform = TransformStamped()

        transform.header.stamp = (
            msg.header.stamp
        )

        transform.header.frame_id = (
            self.odom_frame
        )

        transform.child_frame_id = (
            self.base_frame
        )

        transform.transform.translation.x = (
            odom_x
        )

        transform.transform.translation.y = (
            odom_y
        )

        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0

        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = qz

        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = PriusOdomBridge()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()