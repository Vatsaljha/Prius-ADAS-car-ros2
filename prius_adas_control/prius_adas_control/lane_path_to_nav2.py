#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from rclpy.action import ActionClient

from nav2_msgs.action import FollowPath

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class LanePathToNav2(Node):
    """
    Bridge the locally generated lane path into Nav2 FollowPath.

    Input:
        /adas/lane_path
        nav_msgs/Path
        frame normally = base_link

    Output:
        Nav2 FollowPath action

    The node:
        1. Receives the lane path.
        2. Transforms it from base_link to Nav2's path frame.
        3. Sends it to controller_server.
        4. Uses the existing RPP controller.

    IMPORTANT:
        This node does NOT directly publish /cmd_vel.
        Nav2's controller_server remains responsible for control.
    """

    def __init__(self):

        super().__init__(
            'lane_path_to_nav2'
        )

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter(
            'path_topic',
            '/adas/lane_path'
        )

        self.declare_parameter(
            'nav2_frame',
            'odom'
        )

        self.declare_parameter(
            'controller_id',
            'FollowPath'
        )

        self.declare_parameter(
            'goal_checker_id',
            'goal_checker'
        )

        self.declare_parameter(
            'progress_checker_id',
            'progress_checker'
        )

        self.declare_parameter(
            'update_rate',
            2.0
        )

        self.declare_parameter(
            'min_goal_update_distance',
            0.15
        )

        self.declare_parameter(
            'min_goal_update_angle',
            0.10
        )

        self.declare_parameter(
            'tf_timeout',
            0.20
        )

        # Read parameters

        self.path_topic = str(
            self.get_parameter(
                'path_topic'
            ).value
        )

        self.nav2_frame = str(
            self.get_parameter(
                'nav2_frame'
            ).value
        )

        self.controller_id = str(
            self.get_parameter(
                'controller_id'
            ).value
        )

        self.goal_checker_id = str(
            self.get_parameter(
                'goal_checker_id'
            ).value
        )

        self.progress_checker_id = str(
            self.get_parameter(
                'progress_checker_id'
            ).value
        )

        self.update_rate = float(
            self.get_parameter(
                'update_rate'
            ).value
        )

        self.min_goal_update_distance = float(
            self.get_parameter(
                'min_goal_update_distance'
            ).value
        )

        self.min_goal_update_angle = float(
            self.get_parameter(
                'min_goal_update_angle'
            ).value
        )

        self.tf_timeout = float(
            self.get_parameter(
                'tf_timeout'
            ).value
        )

        # ============================================================
        # STATE
        # ============================================================

        self.latest_path = None

        self.last_sent_path = None

        self.action_active = False

        self.goal_handle = None

        self.goal_in_progress = False

        self.goal_sequence = 0

        self.last_source_frame = ''

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # NAV2 ACTION CLIENT
        # ============================================================

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        # ============================================================
        # PATH SUBSCRIBER
        # ============================================================

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        # ============================================================
        # TIMER
        # ============================================================

        period = 1.0 / max(
            self.update_rate,
            0.1
        )

        self.timer = self.create_timer(
            period,
            self.process_path
        )

        # ============================================================
        # LOGGING
        # ============================================================

        self.received_count = 0
        self.sent_count = 0

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'LANE PATH -> NAV2 BRIDGE STARTED'
        )

        self.get_logger().info(
            f'Input path       : {self.path_topic}'
        )

        self.get_logger().info(
            f'Nav2 path frame  : {self.nav2_frame}'
        )

        self.get_logger().info(
            f'Controller       : {self.controller_id}'
        )

        self.get_logger().info(
            f'Goal checker     : {self.goal_checker_id}'
        )

        self.get_logger().info(
            f'Progress checker : {self.progress_checker_id}'
        )

        self.get_logger().info(
            f'Update rate      : {self.update_rate:.1f} Hz'
        )

        self.get_logger().info(
            '================================================'
        )

    # ================================================================
    # PATH CALLBACK
    # ================================================================

    def path_callback(self, msg):

        if len(msg.poses) < 2:

            self.get_logger().warn(
                'Received lane path with fewer than 2 poses'
            )

            return

        self.latest_path = msg

        self.last_source_frame = (
            msg.header.frame_id
        )

        self.received_count += 1

    # ================================================================
    # PROCESS PATH
    # ================================================================

    def process_path(self):

        if self.latest_path is None:
            return

        # ------------------------------------------------------------
        # Make sure Nav2 action server exists
        # ------------------------------------------------------------

        if not self.follow_path_client.server_is_ready():

            self.get_logger().warn(
                'Waiting for Nav2 FollowPath action server...',
                throttle_duration_sec=3.0
            )

            return

        # ------------------------------------------------------------
        # Transform path
        # ------------------------------------------------------------

        transformed_path = (
            self.transform_path(
                self.latest_path
            )
        )

        if transformed_path is None:
            return

        # ------------------------------------------------------------
        # Decide whether to send a new goal
        # ------------------------------------------------------------

        if self.last_sent_path is not None:

            if not self.path_changed_enough(
                transformed_path,
                self.last_sent_path
            ):

                return

        # ------------------------------------------------------------
        # If an old goal exists, cancel it before replacing it
        # ------------------------------------------------------------

        if self.goal_handle is not None:

            if self.goal_in_progress:

                self.get_logger().info(
                    'Lane path changed - replacing current Nav2 path'
                )

                future = (
                    self.goal_handle.cancel_goal_async()
                )

                future.add_done_callback(
                    lambda f: self.send_new_goal_after_cancel(
                        transformed_path
                    )
                )

                return

        # ------------------------------------------------------------
        # Send new path
        # ------------------------------------------------------------

        self.send_follow_path_goal(
            transformed_path
        )

    # ================================================================
    # TRANSFORM PATH
    # ================================================================

    def transform_path(self, path):

        source_frame = path.header.frame_id

        if source_frame == '':

            self.get_logger().warn(
                'Lane path has empty frame_id'
            )

            return None

        if source_frame == self.nav2_frame:

            return path

        # ------------------------------------------------------------
        # Lookup transform
        # ------------------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                self.nav2_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=self.tf_timeout
                )
            )

        except Exception as exc:

            self.get_logger().warn(
                f'Cannot transform lane path '
                f'{source_frame} -> {self.nav2_frame}: '
                f'{exc}',
                throttle_duration_sec=3.0
            )

            return None

        # ------------------------------------------------------------
        # Build transformed path
        # ------------------------------------------------------------

        result = Path()

        result.header.stamp = (
            self.get_clock().now().to_msg()
        )

        result.header.frame_id = (
            self.nav2_frame
        )

        for pose in path.poses:

            try:

                transformed_pose = (
                    do_transform_pose(
                        pose,
                        transform
                    )
                )

                transformed_pose.header.stamp = (
                    result.header.stamp
                )

                transformed_pose.header.frame_id = (
                    self.nav2_frame
                )

                result.poses.append(
                    transformed_pose
                )

            except Exception as exc:

                self.get_logger().error(
                    f'Failed to transform path pose: {exc}'
                )

                return None

        if len(result.poses) < 2:

            return None

        return result

    # ================================================================
    # CHECK WHETHER PATH CHANGED
    # ================================================================

    def path_changed_enough(
        self,
        new_path,
        old_path
    ):

        if len(new_path.poses) < 2:
            return False

        if len(old_path.poses) < 2:
            return True

        # ------------------------------------------------------------
        # Compare first pose
        # ------------------------------------------------------------

        new_start = (
            new_path.poses[0].pose
        )

        old_start = (
            old_path.poses[0].pose
        )

        dx = (
            new_start.position.x
            -
            old_start.position.x
        )

        dy = (
            new_start.position.y
            -
            old_start.position.y
        )

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        if distance > self.min_goal_update_distance:

            return True

        # ------------------------------------------------------------
        # Compare final pose
        # ------------------------------------------------------------

        new_end = (
            new_path.poses[-1].pose
        )

        old_end = (
            old_path.poses[-1].pose
        )

        end_dx = (
            new_end.position.x
            -
            old_end.position.x
        )

        end_dy = (
            new_end.position.y
            -
            old_end.position.y
        )

        end_distance = math.sqrt(
            end_dx * end_dx
            +
            end_dy * end_dy
        )

        if end_distance > self.min_goal_update_distance:

            return True

        # ------------------------------------------------------------
        # Compare final heading
        # ------------------------------------------------------------

        new_yaw = self.quaternion_to_yaw(
            new_end.orientation
        )

        old_yaw = self.quaternion_to_yaw(
            old_end.orientation
        )

        angle_difference = abs(
            self.normalize_angle(
                new_yaw - old_yaw
            )
        )

        if angle_difference > self.min_goal_update_angle:

            return True

        return False

    # ================================================================
    # SEND NAV2 FOLLOW PATH
    # ================================================================

    def send_follow_path_goal(
        self,
        path
    ):

        goal_msg = FollowPath.Goal()

        goal_msg.path = path

        goal_msg.controller_id = (
            self.controller_id
        )

        goal_msg.goal_checker_id = (
            self.goal_checker_id
        )

        goal_msg.progress_checker_id = (
            self.progress_checker_id
        )

        self.goal_sequence += 1

        sequence = self.goal_sequence

        self.get_logger().info(
            f'Sending lane path to Nav2 '
            f'(goal #{sequence}, '
            f'{len(path.poses)} poses)'
        )

        future = (
            self.follow_path_client
            .send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
        )

        future.add_done_callback(
            lambda f: self.goal_response_callback(
                f,
                path,
                sequence
            )
        )

    # ================================================================
    # SEND NEW GOAL AFTER CANCELLATION
    # ================================================================

    def send_new_goal_after_cancel(
        self,
        path
    ):

        self.goal_handle = None

        self.goal_in_progress = False

        self.send_follow_path_goal(
            path
        )

    # ================================================================
    # GOAL RESPONSE
    # ================================================================

    def goal_response_callback(
        self,
        future,
        path,
        sequence
    ):

        try:

            goal_handle = future.result()

        except Exception as exc:

            self.get_logger().error(
                f'Nav2 FollowPath goal failed: {exc}'
            )

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                f'Nav2 rejected lane path '
                f'goal #{sequence}'
            )

            self.goal_handle = None
            self.goal_in_progress = False

            return

        self.goal_handle = goal_handle

        self.goal_in_progress = True

        self.last_sent_path = path

        self.sent_count += 1

        self.get_logger().info(
            f'Nav2 accepted lane path '
            f'goal #{sequence}'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )

    # ================================================================
    # RESULT
    # ================================================================

    def goal_result_callback(
        self,
        future
    ):

        self.goal_in_progress = False

        try:

            result = future.result()

            status = result.status

            self.get_logger().info(
                f'Nav2 FollowPath finished '
                f'with status {status}'
            )

        except Exception as exc:

            self.get_logger().error(
                f'Error receiving Nav2 result: {exc}'
            )

        self.goal_handle = None

    # ================================================================
    # FEEDBACK
    # ================================================================

    def feedback_callback(
        self,
        feedback_msg
    ):

        feedback = feedback_msg.feedback

        # Nav2 FollowPath feedback contains
        # distance_to_goal and speed in standard Nav2 versions.

        try:

            distance = (
                feedback.distance_to_goal
            )

            speed = (
                feedback.speed
            )

            self.get_logger().debug(
                f'Nav2 lane following | '
                f'distance={distance:.2f} m | '
                f'speed={speed:.2f} m/s'
            )

        except AttributeError:

            pass

    # ================================================================
    # QUATERNION -> YAW
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = (
            2.0 *
            (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0 *
            (
                q.y * q.y
                +
                q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ================================================================
    # NORMALIZE ANGLE
    # ================================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:

            angle -= (
                2.0 * math.pi
            )

        while angle < -math.pi:

            angle += (
                2.0 * math.pi
            )

        return angle


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = LanePathToNav2()

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