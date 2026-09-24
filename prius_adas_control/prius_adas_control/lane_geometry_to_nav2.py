#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


class LaneGeometryToNav2(Node):
    """
    V21 turn-safe map-guided lane + goal + obstacle-avoidance supervisor.

    User workflow:
      1. Start Gazebo/Prius/lane detection.
      2. Start SLAM Toolbox localization.
      3. In RViz set the INITIAL pose with "2D Pose Estimate".
      4. In RViz set the FINAL goal with "2D Goal Pose".
      5. In normal driving, follow the camera-derived lane path.
      6. When a front obstacle is detected:
           - cancel lane following,
           - use the saved global route (toward the RViz goal) to
             choose a lane rejoin point ahead,
           - ask Smac Hybrid-A* to plan from the current pose to that
             rejoin point around the live costmap obstacle,
           - follow that Nav2 detour.
      7. When the obstacle is clear, cancel the detour and automatically
         return to camera lane following.
      8. If the obstacle is dangerously close or critical data are stale,
         request autonomous braking.

    The FINAL GOAL is only a destination / route anchor. It is not used
    as a continuously refreshed local goal, so the lane controller is
    not fighting Nav2 with repeated short NavigateToPose goals.
    """

    def __init__(self):
        super().__init__("lane_geometry_to_nav2")

        # Frames / topics
        self.declare_parameter("input_frame", "base_link")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("lane_path_topic", "/adas/lane_path")
        self.declare_parameter("global_path_topic", "/adas/global_route")
        self.declare_parameter("detour_path_topic", "/adas/nav2_detour_path")
        self.declare_parameter("mode_topic", "/adas/drive_mode")
        self.declare_parameter("emergency_brake_topic", "/adas/emergency_brake")
        self.declare_parameter("goal_topic", "/goal_pose")

        # Lane generation
        self.declare_parameter("path_length", 8.0)
        self.declare_parameter("path_points", 50)
        self.declare_parameter("lane_width", 1.80)
        self.declare_parameter("lane_refresh_period", 2.0)
        self.declare_parameter("lane_timeout", 1.0)
        self.declare_parameter("lateral_error_sign", -1.0)
        self.declare_parameter("heading_error_sign", -1.0)
        self.declare_parameter("max_lane_lateral_offset", 0.55)
        self.declare_parameter("max_lane_heading", 0.40)
        self.declare_parameter("max_lane_curvature", 0.16)
        self.declare_parameter("lane_change_threshold", 0.04)

        # Map-guided lane following.
        # The global Nav2 route is the geometric backbone through turns.
        # Camera lane errors only make a small lateral correction around it.
        self.declare_parameter("map_lane_horizon", 7.0)
        self.declare_parameter("map_lane_points", 55)
        self.declare_parameter("max_camera_lane_offset", 0.30)
        self.declare_parameter("camera_lane_gain", 0.55)
        self.declare_parameter("camera_heading_gain", 0.30)
        self.declare_parameter("max_camera_heading_adjust", 0.12)

        # TF
        self.declare_parameter("tf_timeout", 0.50)

        # Final-goal behavior
        self.declare_parameter("goal_tolerance", 1.5)
        self.declare_parameter("final_path_refresh_period", 3.0)

        # Obstacle / detour behavior
        self.declare_parameter("nav2_obstacle_distance", 6.0)
        self.declare_parameter("obstacle_half_angle", 0.45)
        self.declare_parameter("obstacle_corridor_half_width", 0.85)
        self.declare_parameter("obstacle_min_cluster_points", 4)
        self.declare_parameter("obstacle_clear_time", 2.0)
        self.declare_parameter("rejoin_distance", 10.0)
        self.declare_parameter("rejoin_max_distance", 18.0)
        self.declare_parameter("replan_period", 8.0)
        self.declare_parameter("replan_when_controller_stopped", True)
        self.declare_parameter("controller_stop_replan_time", 1.5)

        # Autonomous braking
        self.declare_parameter("emergency_brake_distance", 0.90)
        self.declare_parameter("emergency_release_distance", 1.80)
        self.declare_parameter("scan_timeout", 0.70)
        self.declare_parameter("planner_failure_brake", True)

        self.input_frame = str(self.get_parameter("input_frame").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.lane_path_topic = str(self.get_parameter("lane_path_topic").value)
        self.global_path_topic = str(self.get_parameter("global_path_topic").value)
        self.detour_path_topic = str(self.get_parameter("detour_path_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.emergency_brake_topic = str(
            self.get_parameter("emergency_brake_topic").value
        )
        self.goal_topic = str(self.get_parameter("goal_topic").value)

        self.path_length = float(self.get_parameter("path_length").value)
        self.path_points = max(30, int(self.get_parameter("path_points").value))
        self.lane_width = float(self.get_parameter("lane_width").value)
        self.lane_refresh_period = float(
            self.get_parameter("lane_refresh_period").value
        )
        self.lane_timeout = float(self.get_parameter("lane_timeout").value)
        self.lateral_error_sign = float(
            self.get_parameter("lateral_error_sign").value
        )
        self.heading_error_sign = float(
            self.get_parameter("heading_error_sign").value
        )
        self.max_lane_lateral_offset = float(
            self.get_parameter("max_lane_lateral_offset").value
        )
        self.max_lane_heading = float(
            self.get_parameter("max_lane_heading").value
        )
        self.max_lane_curvature = float(
            self.get_parameter("max_lane_curvature").value
        )
        self.lane_change_threshold = float(
            self.get_parameter("lane_change_threshold").value
        )

        self.map_lane_horizon = float(
            self.get_parameter("map_lane_horizon").value
        )
        self.map_lane_points = max(
            25,
            int(self.get_parameter("map_lane_points").value),
        )
        self.max_camera_lane_offset = float(
            self.get_parameter("max_camera_lane_offset").value
        )
        self.camera_lane_gain = float(
            self.get_parameter("camera_lane_gain").value
        )
        self.camera_heading_gain = float(
            self.get_parameter("camera_heading_gain").value
        )
        self.max_camera_heading_adjust = float(
            self.get_parameter("max_camera_heading_adjust").value
        )

        self.tf_timeout = float(self.get_parameter("tf_timeout").value)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.final_path_refresh_period = float(
            self.get_parameter("final_path_refresh_period").value
        )

        self.nav2_obstacle_distance = float(
            self.get_parameter("nav2_obstacle_distance").value
        )
        self.obstacle_half_angle = float(
            self.get_parameter("obstacle_half_angle").value
        )
        self.obstacle_corridor_half_width = float(
            self.get_parameter("obstacle_corridor_half_width").value
        )
        self.obstacle_min_cluster_points = max(1, int(
            self.get_parameter("obstacle_min_cluster_points").value
        ))
        self.obstacle_clear_time = float(
            self.get_parameter("obstacle_clear_time").value
        )
        self.rejoin_distance = float(
            self.get_parameter("rejoin_distance").value
        )
        self.rejoin_max_distance = float(
            self.get_parameter("rejoin_max_distance").value
        )
        self.replan_period = float(self.get_parameter("replan_period").value)
        self.replan_when_controller_stopped = bool(
            self.get_parameter("replan_when_controller_stopped").value
        )
        self.controller_stop_replan_time = float(
            self.get_parameter("controller_stop_replan_time").value
        )

        self.emergency_brake_distance = float(
            self.get_parameter("emergency_brake_distance").value
        )
        self.emergency_release_distance = float(
            self.get_parameter("emergency_release_distance").value
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.planner_failure_brake = bool(
            self.get_parameter("planner_failure_brake").value
        )

        # Lane state
        self.lateral_error = 0.0
        self.heading_error = 0.0
        self.have_lane_data = False
        self.last_lane_time = 0.0
        self.last_lane_path_lateral = None
        self.last_lane_path_heading = None

        # Initial pose state
        self.initial_pose_received = False

        # Final goal / route state
        self.final_goal: Optional[PoseStamped] = None
        self.global_route: Optional[Path] = None
        self.last_global_plan_time = 0.0
        self.global_plan_in_progress = False

        # LiDAR state
        self.front_obstacle = False
        self.min_front_distance = float("inf")
        self.last_scan_time = 0.0
        self.clear_since = None
        self.last_safe_cmd_time = 0.0
        self.last_safe_linear = 0.0
        self.safe_cmd_zero_since = None

        # Emergency brake
        self.emergency_brake = False

        # Mode
        self.mode = "WAITING_FOR_INITIAL_POSE"

        # FollowPath state
        self.follow_handle = None
        self.follow_cancel_in_progress = False
        self.pending_after_cancel = None
        self.last_follow_start_time = 0.0

        # ComputePathToPose state
        self.compute_handle = None
        self.compute_kind = None  # "FINAL_ROUTE" or "DETOUR"
        self.last_replan_time = 0.0

        # Final route follow state
        self.final_goal_active = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        # Subscribers
        self.create_subscription(
            Float64,
            "/adas/lane_lateral_error",
            self.lateral_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/adas/lane_heading_error",
            self.heading_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/prius/scan",
            self.scan_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            self.safe_cmd_callback,
            10,
        )

        # Publishers
        self.lane_path_pub = self.create_publisher(
            Path,
            self.lane_path_topic,
            10,
        )
        self.global_path_pub = self.create_publisher(
            Path,
            self.global_path_topic,
            10,
        )
        self.detour_path_pub = self.create_publisher(
            Path,
            self.detour_path_topic,
            10,
        )
        self.mode_pub = self.create_publisher(
            String,
            self.mode_topic,
            10,
        )
        self.brake_pub = self.create_publisher(
            Bool,
            self.emergency_brake_topic,
            10,
        )

        # Nav2 actions
        self.compute_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
        )
        self.follow_client = ActionClient(
            self,
            FollowPath,
            "/follow_path",
        )

        self.timer = self.create_timer(0.10, self.update)

        self.publish_mode()
        self.get_logger().info("==============================================")
        self.get_logger().info("Prius ADAS V21 turn-safe lane supervisor started")
        self.get_logger().info("RViz inputs: initial pose + final goal")
        self.get_logger().info(
            "Normal mode: camera lane following"
        )
        self.get_logger().info(
            "Obstacle mode: Smac Hybrid-A* detour to lane rejoin"
        )
        self.get_logger().info(
            f"Obstacle={self.nav2_obstacle_distance:.1f}m, "
            f"Emergency={self.emergency_brake_distance:.2f}m, "
            f"Goal tolerance={self.goal_tolerance:.1f}m"
        )
        self.get_logger().info("==============================================")

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------

    def safe_cmd_callback(self, msg):
        """Track the velocity that actually survived Collision Monitor.

        This is used only to decide whether an already-running detour has
        been held at zero long enough to justify a re-plan. It is not a
        second controller.
        """
        now = time.monotonic()
        self.last_safe_cmd_time = now

        linear = float(msg.linear.x)
        if not math.isfinite(linear):
            linear = 0.0

        self.last_safe_linear = linear

        if abs(linear) <= 0.002:
            if self.safe_cmd_zero_since is None:
                self.safe_cmd_zero_since = now
        else:
            self.safe_cmd_zero_since = None

    def lateral_callback(self, msg):
        self.lateral_error = float(msg.data)
        self.have_lane_data = True
        self.last_lane_time = time.monotonic()

    def heading_callback(self, msg):
        self.heading_error = float(msg.data)
        self.have_lane_data = True
        self.last_lane_time = time.monotonic()

    def initial_pose_callback(self, _msg):
        self.initial_pose_received = True
        self.set_emergency_brake(False)

        if self.mode == "WAITING_FOR_INITIAL_POSE":
            self.set_mode("WAITING_FOR_FINAL_GOAL")
            self.get_logger().info(
                "RViz initial pose received. Waiting for final goal."
            )

    def goal_callback(self, msg):
        if msg is None:
            return

        goal = self.transform_pose_to_map(msg)
        if goal is None:
            self.get_logger().warning(
                "Final goal received but map transform is not available yet."
            )
            return

        self.final_goal = goal
        self.final_goal_active = False
        self.global_route = None
        self.last_global_plan_time = 0.0

        self.get_logger().info(
            "Final RViz goal received. Planning route to the final goal."
        )

        if self.follow_handle is not None:
            self.pending_after_cancel = "PLAN_FINAL"
            self.cancel_follow()
        else:
            self.request_final_route()

    def _bounded_lane_geometry(self):
        """Return a physically bounded local lane target.

        The camera errors are used as a local correction, but the requested
        path is deliberately bounded so a noisy perception value cannot
        generate a path through a wall or outside the lane.
        """
        lateral = max(-1.0, min(1.0, self.lateral_error))
        heading = max(-0.75, min(0.75, self.heading_error))

        target_lateral = (
            self.lateral_error_sign
            * lateral
            * (self.lane_width / 2.0)
        )
        target_lateral = max(
            -self.max_lane_lateral_offset,
            min(self.max_lane_lateral_offset, target_lateral),
        )

        target_heading = (
            self.heading_error_sign
            * heading
        )
        target_heading = max(
            -self.max_lane_heading,
            min(self.max_lane_heading, target_heading),
        )

        return target_lateral, target_heading

    def _lane_y_at_x(self, x):
        length = max(6.0, self.path_length)
        target_lateral, target_heading = self._bounded_lane_geometry()

        slope_end = math.tan(target_heading)

        a = (
            slope_end * length
            - 2.0 * target_lateral
        ) / (length ** 3)

        b = (
            3.0 * target_lateral
            - slope_end * length
        ) / (length ** 2)

        x = max(0.0, min(length, x))
        y = a * x ** 3 + b * x ** 2

        return max(
            -self.max_lane_lateral_offset,
            min(self.max_lane_lateral_offset, y),
        )

    def _route_lane_y_at_x(self, x, route_local):
        """Return the map-route lateral position near a forward x value.

        During a tight turn the old obstacle detector still used a straight-ish
        camera cubic to decide whether a LiDAR return was in the driving lane.
        That made the inside wall of the bend look like a front obstacle.
        Use the actual final Nav2 route instead whenever it is available.
        """
        if not route_local:
            return None

        best_y = None
        best_dx = float("inf")

        for pose in route_local:
            px = pose.pose.position.x
            if px < 0.20 or px > self.nav2_obstacle_distance + 0.50:
                continue

            dx = abs(px - x)
            if dx < best_dx:
                best_dx = dx
                best_y = pose.pose.position.y

        # Do not force a route estimate when the requested x is outside the
        # locally available route segment. The camera path is the fallback.
        if best_y is None or best_dx > 0.75:
            return None

        return best_y

    def scan_callback(self, msg):
        self.last_scan_time = time.monotonic()

        min_front = float("inf")
        consecutive = 0
        max_cluster = 0
        angle = msg.angle_min

        # IMPORTANT: at a curve, use the already planned map route as the
        # obstacle corridor. This prevents the inner turn wall from being
        # classified as a blocking obstacle simply because it is close to the
        # camera-only cubic model.
        route_local = self._global_route_reference_in_base_link()

        for r in msg.ranges:
            candidate = False
            if math.isfinite(r) and msg.range_min < r < self.nav2_obstacle_distance:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if (
                    x >= 0.30
                    and abs(angle) <= self.obstacle_half_angle
                ):
                    lane_y = self._route_lane_y_at_x(x, route_local)
                    if lane_y is None:
                        lane_y = self._lane_y_at_x(x)

                    # Slightly narrower blocking corridor. Collision Monitor
                    # remains the final safety layer for genuinely close
                    # obstacles.
                    corridor = min(
                        self.obstacle_corridor_half_width,
                        0.60,
                    )
                    if abs(y - lane_y) <= corridor:
                        candidate = True

            if candidate:
                consecutive += 1
                max_cluster = max(max_cluster, consecutive)
                min_front = min(min_front, float(r))
            else:
                consecutive = 0

            angle += msg.angle_increment

        self.min_front_distance = min_front
        self.front_obstacle = max_cluster >= self.obstacle_min_cluster_points

        if self.front_obstacle:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = time.monotonic()

    # ------------------------------------------------------------------
    # BASIC STATE
    # ------------------------------------------------------------------

    def set_mode(self, mode):
        if self.mode == mode:
            return

        self.mode = mode
        self.publish_mode()
        self.get_logger().info(f"DRIVE MODE -> {mode}")

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def set_emergency_brake(self, enabled):
        enabled = bool(enabled)

        if enabled == self.emergency_brake:
            return

        self.emergency_brake = enabled

        msg = Bool()
        msg.data = enabled
        self.brake_pub.publish(msg)

        if enabled:
            self.get_logger().error("AUTONOMOUS BRAKE -> ACTIVE")
        else:
            self.get_logger().info("Autonomous brake released")

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------

    def transform_pose_to_map(self, pose):
        try:
            if pose.header.frame_id == self.output_frame:
                out = PoseStamped()
                out.header = pose.header
                out.header.frame_id = self.output_frame
                out.header.stamp = self.get_clock().now().to_msg()
                out.pose = pose.pose
                return out

            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )

            out = do_transform_pose_stamped(
                pose,
                transform,
            )
            out.header.frame_id = self.output_frame
            out.header.stamp = self.get_clock().now().to_msg()
            return out

        except Exception as exc:
            self.get_logger().warning(
                f"Goal TF conversion failed: {exc}"
            )
            return None

    def transform_lane_to_map(self, local_path):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.input_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )

            out = Path()
            out.header.frame_id = self.output_frame
            out.header.stamp = transform.header.stamp

            for pose in local_path.poses:
                transformed = do_transform_pose_stamped(
                    pose,
                    transform,
                )
                transformed.header.frame_id = self.output_frame
                transformed.header.stamp = transform.header.stamp
                out.poses.append(transformed)

            return out

        except Exception as exc:
            self.get_logger().warning(
                f"Lane path TF conversion failed: {exc}"
            )
            return None

    def current_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.input_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )

            pose = PoseStamped()
            pose.header.frame_id = self.output_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose

        except Exception:
            return None

    # ------------------------------------------------------------------
    # PATH GENERATION
    # ------------------------------------------------------------------

    def _global_route_reference_in_base_link(self):
        """Return a short section of the final route in base_link.

        The map route is deliberately used as the geometric reference for
        normal lane following. This prevents the camera-only cubic from
        cutting the inside of tight figure-8 corners. Camera errors then
        provide only a small correction around this map-safe reference.
        """
        if self.global_route is None or len(self.global_route.poses) < 2:
            return None

        current = self.current_map_pose()
        if current is None:
            return None

        poses = self.global_route.poses

        # Find the closest route pose.
        nearest_index = 0
        nearest_distance = float("inf")
        cx = current.pose.position.x
        cy = current.pose.position.y

        for i, pose in enumerate(poses):
            dx = pose.pose.position.x - cx
            dy = pose.pose.position.y - cy
            d2 = dx * dx + dy * dy
            if d2 < nearest_distance:
                nearest_distance = d2
                nearest_index = i

        # Transform map -> base_link at the current time.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.input_frame,
                self.output_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Route reference TF failed: {exc}"
            )
            return None

        local = []
        accumulated = 0.0
        previous = None

        for i in range(nearest_index, len(poses)):
            pose = poses[i]

            if previous is not None:
                accumulated += math.hypot(
                    pose.pose.position.x - previous.pose.position.x,
                    pose.pose.position.y - previous.pose.position.y,
                )

            previous = pose

            if accumulated > self.map_lane_horizon:
                break

            transformed = do_transform_pose_stamped(
                pose,
                transform,
            )
            local.append(transformed)

        if len(local) < 5:
            return None

        return local

    def build_map_guided_lane_path(self):
        """Build the lane path from the map route plus small camera corrections."""
        route_local = self._global_route_reference_in_base_link()
        if route_local is None:
            return None

        path = Path()
        path.header.frame_id = self.input_frame
        path.header.stamp = self.get_clock().now().to_msg()

        lateral = max(-1.0, min(1.0, self.lateral_error))
        heading = max(-0.75, min(0.75, self.heading_error))

        camera_offset = (
            self.lateral_error_sign
            * lateral
            * (self.lane_width / 2.0)
            * self.camera_lane_gain
        )
        camera_offset = max(
            -self.max_camera_lane_offset,
            min(self.max_camera_lane_offset, camera_offset),
        )

        heading_adjust = (
            self.heading_error_sign
            * heading
            * self.camera_heading_gain
        )
        heading_adjust = max(
            -self.max_camera_heading_adjust,
            min(self.max_camera_heading_adjust, heading_adjust),
        )

        # Resample approximately uniformly along the available route section.
        src = route_local
        n = min(self.map_lane_points, len(src))

        for i in range(n):
            src_pose = src[min(
                len(src) - 1,
                int(i * (len(src) - 1) / float(n - 1)),
            )]

            x = src_pose.pose.position.x
            y = src_pose.pose.position.y

            # Extract route yaw.
            q = src_pose.pose.orientation
            route_yaw = math.atan2(
                2.0 * (q.w * q.z),
                1.0 - 2.0 * (q.z * q.z),
            )

            # Shift only a bounded amount around the map-safe route.
            # Near the vehicle the correction is strong; farther out it
            # smoothly decays so the planned route geometry is retained.
            t = i / float(max(1, n - 1))
            gain = 1.0 - 0.55 * t
            offset = camera_offset * gain

            nx = -math.sin(route_yaw)
            ny = math.cos(route_yaw)

            x += offset * nx
            y += offset * ny

            corrected_yaw = route_yaw + heading_adjust * (1.0 - t)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(corrected_yaw / 2.0)
            pose.pose.orientation.w = math.cos(corrected_yaw / 2.0)

            path.poses.append(pose)

        return path

    def build_lane_path(self):
        """Generate the driving path used during normal lane following.

        Primary reference:
          final Nav2 route (map geometry)

        Small correction:
          camera lane lateral + heading errors

        This is intentionally not a camera-only cubic. Using the map route as
        the backbone keeps tight figure-8 corners away from the walls while
        still allowing the camera detector to center the Prius in the lane.
        """
        map_guided = self.build_map_guided_lane_path()
        if map_guided is not None and len(map_guided.poses) >= 5:
            return map_guided

        # Fallback only before the final map route is available.
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.input_frame

        length = max(5.0, min(7.0, self.path_length))
        lateral = max(-1.0, min(1.0, self.lateral_error))
        heading = max(
            -self.max_lane_heading,
            min(self.max_lane_heading, self.heading_error),
        )

        target_lateral = (
            self.lateral_error_sign
            * lateral
            * (self.lane_width / 2.0)
            * 0.55
        )
        target_lateral = max(
            -self.max_lane_lateral_offset,
            min(self.max_lane_lateral_offset, target_lateral),
        )

        start_yaw = self.heading_error_sign * heading * 0.75
        start_slope = math.tan(start_yaw)

        end_heading = self.heading_error_sign * heading * 0.20
        end_slope = math.tan(end_heading)

        # Hermite cubic: y(0)=0, y'(0)=start_slope,
        # y(L)=target_lateral, y'(L)=end_slope.
        a = (
            length * start_slope
            + length * end_slope
            - 2.0 * target_lateral
        ) / (length ** 3)

        b = (
            -2.0 * length * start_slope
            - length * end_slope
            + 3.0 * target_lateral
        ) / (length ** 2)

        previous_yaw = start_yaw

        for i in range(self.path_points):
            x = length * i / float(self.path_points - 1)

            y = (
                a * x ** 3
                + b * x ** 2
                + start_slope * x
            )

            dy = (
                3.0 * a * x ** 2
                + 2.0 * b * x
                + start_slope
            )

            ddy = (
                6.0 * a * x
                + 2.0 * b
            )

            # Keep curvature compatible with the Prius turning radius.
            curvature = abs(ddy) / max(
                (1.0 + dy * dy) ** 1.5,
                1e-6,
            )

            max_curvature = min(
                self.max_lane_curvature,
                1.0 / 4.5,
            )

            if curvature > max_curvature:
                # Reduce the slope locally instead of asking RPP for a
                # geometrically impossible turn.
                scale = max_curvature / curvature
                dy *= max(0.25, min(1.0, scale))

            yaw = math.atan2(dy, 1.0)

            max_yaw_step = 0.05
            yaw = max(
                previous_yaw - max_yaw_step,
                min(previous_yaw + max_yaw_step, yaw),
            )
            previous_yaw = yaw

            y = max(
                -self.max_lane_lateral_offset,
                min(self.max_lane_lateral_offset, y),
            )

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        return path

    # ------------------------------------------------------------------
    # ROUTE / GOAL GEOMETRY
    # ------------------------------------------------------------------

    def distance_to_final_goal(self):
        if self.final_goal is None:
            return float("inf")

        current = self.current_map_pose()
        if current is None:
            return float("inf")

        return math.hypot(
            self.final_goal.pose.position.x
            - current.pose.position.x,
            self.final_goal.pose.position.y
            - current.pose.position.y,
        )

    def route_rejoin_goal(self):
        """
        Select a point on the global route some distance ahead of the car.

        This is the key hybrid behavior:
          camera -> lane path in normal mode
          Nav2 -> route point on the same final-goal route during obstacle
        """
        if self.global_route is None:
            return None

        current = self.current_map_pose()
        if current is None:
            return None

        poses = self.global_route.poses
        if len(poses) < 2:
            return None

        # Find nearest route pose.
        nearest_index = 0
        nearest_distance = float("inf")

        for i, pose in enumerate(poses):
            d = math.hypot(
                pose.pose.position.x
                - current.pose.position.x,
                pose.pose.position.y
                - current.pose.position.y,
            )

            if d < nearest_distance:
                nearest_distance = d
                nearest_index = i

        # Walk forward along the route until target distance is reached.
        # If the remaining route is shorter than rejoin_distance, keep the
        # furthest usable point instead of declaring the turn unusable.
        accumulated = 0.0
        previous = poses[nearest_index]
        last_usable = None
        last_usable_distance = 0.0

        for i in range(
            nearest_index + 1,
            len(poses),
        ):
            pose = poses[i]

            segment = math.hypot(
                pose.pose.position.x
                - previous.pose.position.x,
                pose.pose.position.y
                - previous.pose.position.y,
            )

            accumulated += segment
            previous = pose

            if accumulated >= 4.0:
                last_usable = pose
                last_usable_distance = accumulated

            if accumulated >= self.rejoin_distance:
                last_usable = pose
                last_usable_distance = accumulated
                break

            if accumulated >= self.rejoin_max_distance:
                break

        if last_usable is not None:
            result = PoseStamped()
            result.header.frame_id = self.output_frame
            result.header.stamp = self.get_clock().now().to_msg()
            result.pose = last_usable.pose

            self.get_logger().info(
                f"Nav2 rejoin point selected "
                f"{last_usable_distance:.1f} m ahead on final route"
            )

            return result

        # Near the end of a route, the final goal itself can be the valid
        # detour target. This avoids a needless emergency stop just because
        # there is less than rejoin_distance of route remaining.
        if self.final_goal is not None:
            goal_distance = math.hypot(
                self.final_goal.pose.position.x
                - current.pose.position.x,
                self.final_goal.pose.position.y
                - current.pose.position.y,
            )
            if goal_distance >= 2.5:
                self.get_logger().info(
                    f"Using final goal as Nav2 rejoin point ({goal_distance:.1f} m)"
                )
                return self.final_goal

        return None

    # ------------------------------------------------------------------
    # COMPUTE PATH
    # ------------------------------------------------------------------

    def request_final_route(self):
        if self.final_goal is None:
            return

        if self.compute_handle is not None or self.global_plan_in_progress:
            return

        if self.follow_cancel_in_progress:
            return

        if not self.compute_client.server_is_ready():
            return

        self.global_plan_in_progress = True

        request = ComputePathToPose.Goal()
        request.goal = self.final_goal
        request.planner_id = "GridBased"
        request.use_start = False

        self.compute_kind = "FINAL_ROUTE"

        future = self.compute_client.send_goal_async(
            request
        )
        future.add_done_callback(
            self.compute_goal_response
        )

        self.last_global_plan_time = time.monotonic()

    def request_detour(self):
        if not self.front_obstacle:
            return

        if self.compute_handle is not None:
            return

        if not self.compute_client.server_is_ready():
            return

        goal = self.route_rejoin_goal()

        # If no global route is available yet, create a local lane rejoin
        # target as a fallback.
        if goal is None and self.have_lane_data:
            lane = self.transform_lane_to_map(
                self.build_lane_path()
            )

            if lane is not None and len(lane.poses) > 3:
                current = self.current_map_pose()

                if current is not None:
                    for pose in lane.poses:
                        d = math.hypot(
                            pose.pose.position.x
                            - current.pose.position.x,
                            pose.pose.position.y
                            - current.pose.position.y,
                        )

                        if d >= self.rejoin_distance:
                            goal = pose
                            break

        if goal is None:
            self.get_logger().warning(
                "No safe Nav2 rejoin point available"
            )

            if self.planner_failure_brake:
                self.set_emergency_brake(True)

            return

        request = ComputePathToPose.Goal()
        request.goal = goal
        request.planner_id = "GridBased"
        request.use_start = False

        self.compute_kind = "DETOUR"
        self.global_plan_in_progress = False
        self.last_replan_time = time.monotonic()

        future = self.compute_client.send_goal_async(
            request
        )
        future.add_done_callback(
            self.compute_goal_response
        )

        self.get_logger().info(
            "Obstacle detected -> requesting Smac Hybrid-A* detour"
        )

    def compute_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.compute_handle = None
            self.global_plan_in_progress = False
            self.get_logger().error(
                f"ComputePathToPose request failed: {exc}"
            )

            if (
                self.compute_kind == "DETOUR"
                and self.front_obstacle
                and self.planner_failure_brake
            ):
                self.set_emergency_brake(True)

            return

        if not handle.accepted:
            self.compute_handle = None
            self.global_plan_in_progress = False
            self.get_logger().warning(
                f"Nav2 planner rejected {self.compute_kind} request"
            )

            if (
                self.compute_kind == "DETOUR"
                and self.front_obstacle
                and self.planner_failure_brake
            ):
                self.set_emergency_brake(True)

            return

        self.compute_handle = handle

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            self.compute_result_callback
        )

    def compute_result_callback(self, future):
        kind = self.compute_kind
        self.compute_handle = None
        self.compute_kind = None
        self.global_plan_in_progress = False

        try:
            path = future.result().result.path
        except Exception as exc:
            self.get_logger().error(
                f"Nav2 planner result unavailable: {exc}"
            )

            if (
                kind == "DETOUR"
                and self.front_obstacle
                and self.planner_failure_brake
            ):
                self.set_emergency_brake(True)

            return

        if path is None or len(path.poses) < 2:
            self.get_logger().warning(
                f"Nav2 returned no valid path for {kind}"
            )

            if (
                kind == "DETOUR"
                and self.front_obstacle
                and self.planner_failure_brake
            ):
                self.set_emergency_brake(True)

            return

        path.header.frame_id = self.output_frame
        path.header.stamp = self.get_clock().now().to_msg()

        if kind == "FINAL_ROUTE":
            self.global_route = path
            self.global_path_pub.publish(path)
            self.get_logger().info(
                f"Final-goal route ready: {len(path.poses)} poses"
            )

            if self.mode == "WAITING_FOR_FINAL_GOAL":
                self.set_mode("LANE_FOLLOW")

            return

        if kind == "DETOUR":
            self.detour_path_pub.publish(path)
            self.set_emergency_brake(False)

            if (
                self.mode == "NAV2_DETOUR"
                and self.front_obstacle
            ):
                self.send_follow_path(
                    path,
                    "NAV2_DETOUR",
                )

    # ------------------------------------------------------------------
    # FOLLOW PATH
    # ------------------------------------------------------------------

    def cancel_follow(self, pending=None):
        if self.follow_handle is None:
            return False

        if self.follow_cancel_in_progress:
            return False

        self.follow_cancel_in_progress = True
        self.pending_after_cancel = pending

        try:
            future = self.follow_handle.cancel_goal_async()
            future.add_done_callback(
                self.follow_cancel_done
            )
            return True
        except Exception as exc:
            self.follow_cancel_in_progress = False
            self.pending_after_cancel = None
            self.get_logger().warning(
                f"FollowPath cancel failed: {exc}"
            )
            return False

    def follow_cancel_done(self, _future):
        self.follow_handle = None
        self.follow_cancel_in_progress = False

        pending = self.pending_after_cancel
        self.pending_after_cancel = None

        if pending == "LANE":
            self.send_lane_path()

        elif pending == "DETOUR":
            self.request_detour()

        elif pending == "FINAL":
            self.send_final_path()

        elif pending == "PLAN_FINAL":
            self.request_final_route()

    def send_follow_path(self, path, mode):
        if path is None or len(path.poses) < 3:
            return False

        if not self.follow_client.server_is_ready():
            return False

        if (
            self.follow_handle is not None
            or self.follow_cancel_in_progress
        ):
            return False

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        future = self.follow_client.send_goal_async(
            goal
        )
        future.add_done_callback(
            self.follow_response_callback
        )

        self.last_follow_start_time = time.monotonic()

        self.get_logger().info(
            f"FollowPath request: {mode}, {len(path.poses)} poses"
        )

        return True

    def follow_response_callback(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"FollowPath request failed: {exc}"
            )
            return

        if not handle.accepted:
            self.follow_handle = None
            self.get_logger().warning(
                f"FollowPath rejected in mode {self.mode}"
            )
            return

        self.follow_handle = handle

        self.get_logger().info(
            f"FollowPath accepted: {self.mode}"
        )

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            self.follow_result_callback
        )

    def follow_result_callback(self, future):
        self.follow_handle = None

        try:
            status = int(
                future.result().status
            )
        except Exception as exc:
            self.get_logger().warning(
                f"FollowPath result read failed: {exc}"
            )
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f"FollowPath ended with status {status}"
            )

        # The timer decides the next path/mode.
        # Do not immediately issue a second goal from this callback.

    # ------------------------------------------------------------------
    # LANE FOLLOW
    # ------------------------------------------------------------------

    def send_lane_path(self):
        if self.mode != "LANE_FOLLOW":
            return

        if not self.have_lane_data:
            return

        if (
            time.monotonic()
            - self.last_lane_time
            > self.lane_timeout
        ):
            return

        if self.front_obstacle:
            return

        local_path = self.build_lane_path()
        if len(local_path.poses) < 3:
            return

        map_path = self.transform_lane_to_map(
            local_path
        )

        self.lane_path_pub.publish(
            local_path
        )

        if map_path is None:
            self.set_emergency_brake(True)
            return

        self.set_emergency_brake(False)

        if self.send_follow_path(
            map_path,
            "LANE_FOLLOW",
        ):
            self.last_follow_start_time = time.monotonic()
            self.last_lane_path_lateral = self.lateral_error
            self.last_lane_path_heading = self.heading_error

    # ------------------------------------------------------------------
    # FINAL GOAL
    # ------------------------------------------------------------------

    def send_final_path(self):
        if self.global_route is None:
            self.request_final_route()
            return

        self.set_mode("FINAL_GOAL")

        self.set_emergency_brake(False)

        self.send_follow_path(
            self.global_route,
            "FINAL_GOAL",
        )

    # ------------------------------------------------------------------
    # MAIN
    # ------------------------------------------------------------------

    def update(self):
        now = time.monotonic()
        self.publish_mode()

        # --------------------------------------------------------------
        # Initial pose
        # --------------------------------------------------------------
        if not self.initial_pose_received:
            self.set_mode("WAITING_FOR_INITIAL_POSE")
            return

        # --------------------------------------------------------------
        # Final goal has not been supplied yet.
        # --------------------------------------------------------------
        if self.final_goal is None:
            if self.mode != "WAITING_FOR_FINAL_GOAL":
                self.set_mode("WAITING_FOR_FINAL_GOAL")
            return

        # --------------------------------------------------------------
        # Scan watchdog
        # --------------------------------------------------------------
        scan_stale = (
            now - self.last_scan_time
            > self.scan_timeout
        )

        if scan_stale:
            self.set_emergency_brake(True)

            if self.follow_handle is not None:
                self.cancel_follow()

            return

        # --------------------------------------------------------------
        # Emergency brake
        # --------------------------------------------------------------
        if (
            self.min_front_distance
            <= self.emergency_brake_distance
        ):
            if self.mode != "EMERGENCY_BRAKE":
                self.set_mode("EMERGENCY_BRAKE")

                if self.follow_handle is not None:
                    self.cancel_follow()

            self.set_emergency_brake(True)
            return

        # Release emergency brake after clearance.
        if self.mode == "EMERGENCY_BRAKE":
            if (
                not self.front_obstacle
                and self.clear_since is not None
                and now - self.clear_since
                >= self.obstacle_clear_time
            ):
                self.set_emergency_brake(False)
                self.set_mode("LANE_FOLLOW")

                if self.follow_handle is None:
                    self.send_lane_path()

            return

        # --------------------------------------------------------------
        # Goal reached
        # --------------------------------------------------------------
        final_distance = self.distance_to_final_goal()

        if (
            final_distance <= self.goal_tolerance
            and self.mode != "GOAL_REACHED"
        ):
            if self.follow_handle is not None:
                self.pending_after_cancel = None
                self.cancel_follow()

            self.set_emergency_brake(False)
            self.set_mode("GOAL_REACHED")
            self.get_logger().info(
                "Final RViz goal reached. Vehicle stopped."
            )
            return

        if self.mode == "GOAL_REACHED":
            return

        # --------------------------------------------------------------
        # Final route must exist.
        # --------------------------------------------------------------
        if self.global_route is None:
            if (
                self.compute_handle is None
                and not self.global_plan_in_progress
            ):
                self.request_final_route()

            return

        # --------------------------------------------------------------
        # OBSTACLE -> NAV2 DETOUR
        # --------------------------------------------------------------
        if self.front_obstacle:

            self.clear_since = None

            if self.mode != "NAV2_DETOUR":
                self.set_mode("NAV2_DETOUR")

                if self.follow_handle is not None:
                    self.cancel_follow("DETOUR")
                else:
                    self.request_detour()

                return

            # Keep an accepted detour running. Do not cancel it on a fixed
            # timer; repeated cancel/restart cycles were preventing the car
            # from ever producing a sustained forward command. Replan only
            # when the controller is actually stopped while the obstacle is
            # still present.
            if self.follow_handle is not None:
                if (
                    self.replan_when_controller_stopped
                    and self.safe_cmd_zero_since is not None
                    and now - self.safe_cmd_zero_since
                    >= self.controller_stop_replan_time
                    and now - self.last_replan_time
                    >= self.replan_period
                    and not self.follow_cancel_in_progress
                ):
                    self.cancel_follow("DETOUR")
                return

            if self.compute_handle is None:
                self.request_detour()

            return

        # --------------------------------------------------------------
        # OBSTACLE CLEARED -> RETURN TO LANE
        # --------------------------------------------------------------
        if self.mode == "NAV2_DETOUR":
            if (
                self.clear_since is not None
                and now - self.clear_since
                >= self.obstacle_clear_time
            ):
                self.set_mode("LANE_FOLLOW")

                if self.follow_handle is not None:
                    self.cancel_follow("LANE")
                else:
                    self.send_lane_path()

            return

        # --------------------------------------------------------------
        # NORMAL LANE FOLLOW
        #
        # The global route is a reference for turns and detours; it is not
        # allowed to replace the camera lane path during the last 5 m.
        # --------------------------------------------------------------
        # --------------------------------------------------------------
        if self.mode != "LANE_FOLLOW":
            self.set_mode("LANE_FOLLOW")

        lane_stale = (
            not self.have_lane_data
            or now - self.last_lane_time
            > self.lane_timeout
        )

        if lane_stale:
            self.set_emergency_brake(True)

            if self.follow_handle is not None:
                self.cancel_follow()

            return

        self.set_emergency_brake(False)

        if self.follow_handle is None:
            self.send_lane_path()
            return

        # Refresh only when the lane estimate has changed enough or the
        # current lane path has become old. This avoids repeatedly cancelling
        # FollowPath while the vehicle is already tracking a stable path.
        lateral_change = 0.0
        heading_change = 0.0
        if self.last_lane_path_lateral is not None:
            lateral_change = abs(
                self.lateral_error - self.last_lane_path_lateral
            )
            heading_change = abs(
                self.heading_error - self.last_lane_path_heading
            )

        meaningful_lane_change = (
            lateral_change >= self.lane_change_threshold
            or heading_change >= self.lane_change_threshold
        )

        if (
            not self.follow_cancel_in_progress
            and now - self.last_follow_start_time
            >= self.lane_refresh_period
            and meaningful_lane_change
        ):
            self.cancel_follow("LANE")



def main(args=None):
    rclpy.init(args=args)

    node = LaneGeometryToNav2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.set_emergency_brake(True)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()