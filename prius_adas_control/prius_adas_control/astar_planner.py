#!/usr/bin/env python3

import heapq
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class AStarPlanner(Node):

    def __init__(self):
        super().__init__("astar_planner")

        self.map_msg = None
        self.odom_msg = None
        self.goal_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/astar_goal",
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            "/astar_path",
            10
        )

        self.get_logger().info(
            "A* planner started."
        )

        self.get_logger().info(
            "Waiting for /map, /odom and /astar_goal."
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def goal_callback(self, msg):

        if msg.header.frame_id == "map":
            self.goal_pose = msg

        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    msg.header.frame_id,
                    rclpy.time.Time()
                )

                transformed = do_transform_pose(
                    msg,
                    transform
                )

                self.goal_pose = transformed

            except Exception as exc:

                self.get_logger().error(
                    f"Could not transform goal to map: {exc}"
                )

                return

        self.get_logger().info(
            "New A* goal received."
        )

        self.plan_path()

    # =========================================================
    # CURRENT ROBOT POSE
    # =========================================================

    def get_robot_pose_in_map(self):

        if self.odom_msg is None:
            return None

        try:

            pose = PoseStamped()

            pose.header = self.odom_msg.header
            pose.header.frame_id = "odom"

            pose.pose = self.odom_msg.pose.pose

            transform = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time()
            )

            transformed = do_transform_pose(
                pose,
                transform
            )

            return transformed

        except Exception as exc:

            self.get_logger().warn(
                f"Unable to transform odom to map: {exc}"
            )

            return None

    # =========================================================
    # WORLD -> MAP CELL
    # =========================================================

    def world_to_grid(self, x, y):

        info = self.map_msg.info

        gx = int(
            (x - info.origin.position.x)
            / info.resolution
        )

        gy = int(
            (y - info.origin.position.y)
            / info.resolution
        )

        return gx, gy

    # =========================================================
    # MAP CELL -> WORLD
    # =========================================================

    def grid_to_world(self, gx, gy):

        info = self.map_msg.info

        x = (
            info.origin.position.x
            + (gx + 0.5) * info.resolution
        )

        y = (
            info.origin.position.y
            + (gy + 0.5) * info.resolution
        )

        return x, y

    # =========================================================
    # VALID CELL
    # =========================================================

    def is_free(self, gx, gy):

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        if gx < 0 or gx >= width:
            return False

        if gy < 0 or gy >= height:
            return False

        index = gy * width + gx

        value = self.map_msg.data[index]

        # Unknown is treated as blocked.
        if value < 0:
            return False

        # Occupied cells.
        if value >= 50:
            return False

        return True

    # =========================================================
    # A*
    # =========================================================

    def heuristic(self, a, b):

        return math.hypot(
            a[0] - b[0],
            a[1] - b[1]
        )

    def astar(self, start, goal):

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start
            )
        )

        came_from = {}

        g_cost = {
            start: 0.0
        }

        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]

        while open_set:

            _, current = heapq.heappop(
                open_set
            )

            if current == goal:

                path = []

                while current in came_from:

                    path.append(current)

                    current = came_from[current]

                path.append(start)

                path.reverse()

                return path

            for dx, dy in directions:

                neighbor = (
                    current[0] + dx,
                    current[1] + dy
                )

                if not self.is_free(
                    neighbor[0],
                    neighbor[1]
                ):
                    continue

                if dx != 0 and dy != 0:
                    step_cost = math.sqrt(2.0)

                else:
                    step_cost = 1.0

                new_cost = (
                    g_cost[current]
                    + step_cost
                )

                if (
                    neighbor not in g_cost
                    or new_cost < g_cost[neighbor]
                ):

                    g_cost[neighbor] = new_cost

                    priority = (
                        new_cost
                        + self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            priority,
                            neighbor
                        )
                    )

                    came_from[neighbor] = current

        return None

    # =========================================================
    # PLAN
    # =========================================================

    def plan_path(self):

        if self.map_msg is None:

            self.get_logger().warn(
                "No map received."
            )

            return

        robot_pose = (
            self.get_robot_pose_in_map()
        )

        if robot_pose is None:

            self.get_logger().warn(
                "Robot pose unavailable."
            )

            return

        start = self.world_to_grid(
            robot_pose.pose.position.x,
            robot_pose.pose.position.y
        )

        goal = self.world_to_grid(
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y
        )

        self.get_logger().info(
            f"A* start: {start}"
        )

        self.get_logger().info(
            f"A* goal: {goal}"
        )

        if not self.is_free(
            start[0],
            start[1]
        ):

            self.get_logger().error(
                "Robot start cell is occupied."
            )

            return

        if not self.is_free(
            goal[0],
            goal[1]
        ):

            self.get_logger().error(
                "Goal cell is occupied."
            )

            return

        grid_path = self.astar(
            start,
            goal
        )

        if grid_path is None:

            self.get_logger().error(
                "A* could not find a path."
            )

            return

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        path_msg.header.frame_id = "map"

        for gx, gy in grid_path:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            pose = PoseStamped()

            pose.header = path_msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(
                pose
            )

        self.path_pub.publish(
            path_msg
        )

        self.get_logger().info(
            f"A* path generated: "
            f"{len(path_msg.poses)} cells."
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
