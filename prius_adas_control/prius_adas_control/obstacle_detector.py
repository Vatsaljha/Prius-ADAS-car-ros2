#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, Float64


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__("obstacle_detector")

        # =====================================================
        # LiDAR
        # =====================================================

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/prius/scan",
            self.lidar_callback,
            20,
        )

        # =====================================================
        # FOUR FRONT SONARS
        # =====================================================

        self.left_far_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/left_far_range",
            self.left_far_callback,
            20,
        )

        self.left_middle_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/left_middle_range",
            self.left_middle_callback,
            20,
        )

        self.right_far_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/right_far_range",
            self.right_far_callback,
            20,
        )

        self.right_middle_sub = self.create_subscription(
            Range,
            "/prius/front_sonar/right_middle_range",
            self.right_middle_callback,
            20,
        )

        # =====================================================
        # ODOMETRY
        # =====================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            20,
        )

        # =====================================================
        # STATIC MAP + TF
        # =====================================================
        #
        # The track barriers are permanent world geometry and are
        # already present in the saved /map.  We therefore classify
        # a sensor return as a *static map obstacle* first.  Static
        # map returns are not treated as a new obstacle.  New objects
        # that are not present in the map remain valid obstacles.
        #
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        # =====================================================
        # OUTPUTS
        # =====================================================

        self.distance_pub = self.create_publisher(
            Float64,
            "/adas/obstacle_distance",
            10,
        )

        self.detected_pub = self.create_publisher(
            Bool,
            "/adas/obstacle_detected",
            10,
        )

        self.brake_pub = self.create_publisher(
            Float64,
            "/adas/brake",
            10,
        )

        # =====================================================
        # SENSOR VALUES
        # =====================================================

        self.lidar_distance = float("inf")
        self.lidar_static_distance = float("inf")

        # Latest static occupancy map.
        self.static_map = None
        self.last_map_time = 0.0

        # Only occupied cells are treated as known static geometry.
        self.map_occupied_threshold = 65
        self.map_match_radius = 0.25
        self.map_min_occupied_neighbors = 2

        # Sensor frame names from the supplied Prius URDF.  The
        # callback uses the message frame_id when available.
        self.sonar_frame_ids = {
            "left_far": "front_left_far_sonar_link",
            "left_middle": "front_left_middle_sonar_link",
            "right_far": "front_right_far_sonar_link",
            "right_middle": "front_right_middle_sonar_link",
        }

        self.left_far = float("inf")
        self.left_middle = float("inf")
        self.right_far = float("inf")
        self.right_middle = float("inf")

        self.vehicle_speed = 0.0

        # =====================================================
        # SENSOR TIMESTAMPS
        # =====================================================

        self.last_lidar_time = 0.0

        self.last_left_far_time = 0.0
        self.last_left_middle_time = 0.0
        self.last_right_far_time = 0.0
        self.last_right_middle_time = 0.0

        self.last_odom_time = 0.0

        # =====================================================
        # FRONT LiDAR = 180 DEGREES
        # =====================================================

        self.lidar_min_angle_deg = -90.0
        self.lidar_max_angle_deg = 90.0

        # =====================================================
        # LiDAR VALID RANGE
        # =====================================================

        self.lidar_min_valid = 0.20
        self.lidar_max_valid = 30.0

        # =====================================================
        # NAV2 HANDOVER
        #
        # Detection should happen early enough for Nav2
        # to receive control and replan.
        # =====================================================

        self.nav2_handover_time = 1.50
        self.nav2_handover_margin = 1.00

        self.nav2_handover_min_distance = 3.0
        self.nav2_handover_max_distance = 6.0

        # =====================================================
        # SONAR MAXIMUM USEFUL DETECTION RANGE
        #
        # The actual sonar max range from your data is 5 m.
        # =====================================================

        self.sonar_max_detection_distance = 5.0

        # =====================================================
        # LiDAR CLUSTERING
        # =====================================================

        self.lidar_cluster_min_points = 3

        self.lidar_cluster_max_gap_deg = 3.0

        # =====================================================
        # SENSOR FRESHNESS
        # =====================================================

        self.sensor_timeout = 0.30

        # =====================================================
        # TEMPORAL CONFIRMATION
        #
        # All sensors have equal authority here.
        #
        # A single sensor can trigger detection, but it must
        # remain detected for consecutive cycles.
        # =====================================================

        self.confirmation_cycles = 2
        self.clear_cycles = 5

        self.obstacle_hold_time = 0.40

        # =====================================================
        # EMERGENCY BRAKING
        #
        # Final close-range safety layer only.
        # Normal avoidance is handled by Nav2.
        # =====================================================

        self.brake_reaction_time = 0.35
        self.brake_deceleration = 3.0
        self.brake_margin = 0.30

        self.minimum_brake_distance = 0.60
        self.maximum_brake_distance = 2.00

        self.brake_hold_time = 0.50

        # =====================================================
        # STATE
        # =====================================================

        self.obstacle_active = False

        self.detect_count = 0
        self.clear_count = 0

        self.last_obstacle_seen_time = 0.0

        self.brake_until = 0.0

        self.last_status = ""

        self.last_detection_source = ""

        # =====================================================
        # LOOP
        # =====================================================

        self.loop_hz = 20.0

        self.timer = self.create_timer(
            1.0 / self.loop_hz,
            self.control_loop,
        )

        # =====================================================
        # STARTUP
        # =====================================================

        self.get_logger().info(
            "================================================"
        )

        self.get_logger().info(
            "MAP-AWARE LIDAR + 4-SONAR DETECTOR"
        )

        self.get_logger().info(
            "LiDAR field: -90 deg to +90 deg"
        )

        self.get_logger().info(
            "All five sensors have equal obstacle authority"
        )

        self.get_logger().info(
            "Static track barriers are removed using /map"
        )

        self.get_logger().info(
            "Obstacle detection -> Nav2 takeover"
        )

        self.get_logger().info(
            "Emergency brake -> final safety layer"
        )

        self.get_logger().info(
            "================================================"
        )

    # =========================================================
    # STATIC MAP CALLBACK
    # =========================================================

    def map_callback(self, msg):

        self.static_map = msg
        self.last_map_time = time.monotonic()

    # =========================================================
    # MAP HEALTH
    # =========================================================

    def map_is_ready(self):

        if self.static_map is None:
            return False

        if self.static_map.info.resolution <= 0.0:
            return False

        if self.static_map.info.width <= 0:
            return False

        if self.static_map.info.height <= 0:
            return False

        return True

    # =========================================================
    # MAP COORDINATE CONVERSION
    # =========================================================

    def map_xy_to_cell(self, x, y):

        if not self.map_is_ready():
            return None

        info = self.static_map.info

        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)

        qx = float(info.origin.orientation.x)
        qy = float(info.origin.orientation.y)
        qz = float(info.origin.orientation.z)
        qw = float(info.origin.orientation.w)

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        dx = x - origin_x
        dy = y - origin_y

        c = math.cos(-yaw)
        s = math.sin(-yaw)

        mx = dx * c - dy * s
        my = dx * s + dy * c

        resolution = float(info.resolution)

        gx = int(math.floor(mx / resolution))
        gy = int(math.floor(my / resolution))

        if gx < 0 or gy < 0:
            return None

        if gx >= int(info.width) or gy >= int(info.height):
            return None

        return gx, gy

    # =========================================================
    # STATIC MAP OCCUPANCY TEST
    # =========================================================

    def point_is_static_map_geometry(self, x, y):

        if not self.map_is_ready():
            return False

        info = self.static_map.info
        cell = self.map_xy_to_cell(x, y)

        if cell is None:
            return False

        gx, gy = cell
        radius_cells = max(
            1,
            int(
                math.ceil(
                    self.map_match_radius
                    /
                    float(info.resolution)
                )
            )
        )

        occupied_neighbors = 0

        for oy in range(
            -radius_cells,
            radius_cells + 1,
        ):

            for ox in range(
                -radius_cells,
                radius_cells + 1,
            ):

                if (
                    ox * ox
                    +
                    oy * oy
                    >
                    radius_cells * radius_cells
                ):
                    continue

                nx = gx + ox
                ny = gy + oy

                if nx < 0 or ny < 0:
                    continue

                if nx >= int(info.width) or ny >= int(info.height):
                    continue

                index = (
                    ny * int(info.width)
                    +
                    nx
                )

                if index < 0 or index >= len(self.static_map.data):
                    continue

                value = int(self.static_map.data[index])

                if (
                    value >= self.map_occupied_threshold
                ):
                    occupied_neighbors += 1

                    if (
                        occupied_neighbors
                        >=
                        self.map_min_occupied_neighbors
                    ):
                        return True

        return False

    # =========================================================
    # TF MAP TRANSFORM
    # =========================================================

    def lookup_map_transform(
        self,
        source_frame,
    ):

        if not self.map_is_ready():
            return None

        if not source_frame:
            return None

        try:

            return self.tf_buffer.lookup_transform(
                self.static_map.header.frame_id or "map",
                source_frame,
                Time(),
                timeout=Duration(
                    seconds=0.05
                ),
            )

        except TransformException:
            return None

    def apply_transform(
        self,
        transform,
        x,
        y,
        z=0.0,
    ):

        t = transform.transform

        qx = float(t.rotation.x)
        qy = float(t.rotation.y)
        qz = float(t.rotation.z)
        qw = float(t.rotation.w)

        # Quaternion rotation: q * p * q^-1.
        tx = 2.0 * (
            qy * z
            -
            qz * y
        )

        ty = 2.0 * (
            qz * x
            -
            qx * z
        )

        tz = 2.0 * (
            qx * y
            -
            qy * x
        )

        rx = x + qw * tx + (qy * tz - qz * ty)
        ry = y + qw * ty + (qz * tx - qx * tz)
        rz = z + qw * tz + (qx * ty - qy * tx)

        return (
            rx + float(t.translation.x),
            ry + float(t.translation.y),
            rz + float(t.translation.z),
        )

    def transform_point_to_map(
        self,
        source_frame,
        x,
        y,
        z=0.0,
    ):

        transform = self.lookup_map_transform(
            source_frame
        )

        if transform is None:
            return None

        return self.apply_transform(
            transform,
            x,
            y,
            z,
        )

    # =========================================================
    # LiDAR CALLBACK
    # =========================================================

    def lidar_callback(self, msg):

        self.last_lidar_time = time.monotonic()

        if not self.map_is_ready():

            self.lidar_distance = float("inf")
            self.lidar_static_distance = float("inf")
            return

        source_frame = (
            msg.header.frame_id
            if msg.header.frame_id
            else ""
        )

        transform = self.lookup_map_transform(
            source_frame
        )

        if transform is None:

            self.lidar_distance = float("inf")
            self.lidar_static_distance = float("inf")
            return

        valid_points = []

        lower_limit = max(
            float(msg.range_min),
            self.lidar_min_valid,
        )

        upper_limit = min(
            float(msg.range_max),
            self.lidar_max_valid,
        )

        for index, raw_distance in enumerate(
            msg.ranges
        ):

            try:
                distance = float(raw_distance)
            except Exception:
                continue

            if not math.isfinite(distance):
                continue

            if distance < lower_limit:
                continue

            if distance > upper_limit:
                continue

            angle = (
                msg.angle_min
                +
                index * msg.angle_increment
            )

            angle_deg = math.degrees(angle)

            if (
                angle_deg < self.lidar_min_angle_deg
                or
                angle_deg > self.lidar_max_angle_deg
            ):
                continue

            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)

            map_point = self.apply_transform(
                transform,
                local_x,
                local_y,
                0.0,
            )

            if map_point is None:
                continue

            map_x = map_point[0]
            map_y = map_point[1]

            static_geometry = (
                self.point_is_static_map_geometry(
                    map_x,
                    map_y,
                )
            )

            valid_points.append(
                (
                    distance,
                    static_geometry,
                )
            )

        if not valid_points:

            self.lidar_distance = float("inf")
            self.lidar_static_distance = float("inf")
            return

        dynamic_distances = []
        static_distances = []

        for distance, static_geometry in valid_points:

            if static_geometry:
                static_distances.append(distance)
            else:
                dynamic_distances.append(distance)

        self.lidar_distance = (
            min(dynamic_distances)
            if dynamic_distances
            else float("inf")
        )

        self.lidar_static_distance = (
            min(static_distances)
            if static_distances
            else float("inf")
        )

    # =========================================================
    # SONAR CALLBACKS
    # =========================================================

    def left_far_callback(self, msg):

        if msg.header.frame_id:

            self.sonar_frame_ids["left_far"] = msg.header.frame_id

        self.left_far = (
            self.valid_sonar_range(msg)
        )

        self.last_left_far_time = (
            time.monotonic()
        )

    def left_middle_callback(self, msg):

        if msg.header.frame_id:

            self.sonar_frame_ids["left_middle"] = msg.header.frame_id

        self.left_middle = (
            self.valid_sonar_range(msg)
        )

        self.last_left_middle_time = (
            time.monotonic()
        )

    def right_far_callback(self, msg):

        if msg.header.frame_id:

            self.sonar_frame_ids["right_far"] = msg.header.frame_id

        self.right_far = (
            self.valid_sonar_range(msg)
        )

        self.last_right_far_time = (
            time.monotonic()
        )

    def right_middle_callback(self, msg):

        if msg.header.frame_id:

            self.sonar_frame_ids["right_middle"] = msg.header.frame_id

        self.right_middle = (
            self.valid_sonar_range(msg)
        )

        self.last_right_middle_time = (
            time.monotonic()
        )

    # =========================================================
    # ODOMETRY CALLBACK
    # =========================================================

    def odom_callback(self, msg):

        vx = float(
            msg.twist.twist.linear.x
        )

        vy = float(
            msg.twist.twist.linear.y
        )

        self.vehicle_speed = math.sqrt(
            vx * vx
            +
            vy * vy
        )

        self.last_odom_time = (
            time.monotonic()
        )

    # =========================================================
    # SONAR VALIDATION
    # =========================================================

    def valid_sonar_range(self, msg):

        distance = float(
            msg.range
        )

        if not math.isfinite(distance):

            return float("inf")

        if distance < float(
            msg.min_range
        ):

            return float("inf")

        if distance > float(
            msg.max_range
        ):

            return float("inf")

        return distance

    # =========================================================
    # SENSOR FRESHNESS
    # =========================================================

    def sensor_is_fresh(
        self,
        timestamp
    ):

        if timestamp <= 0.0:

            return False

        return (
            time.monotonic()
            -
            timestamp
            <=
            self.sensor_timeout
        )

    # =========================================================
    # FRESH SENSOR VALUES
    # =========================================================

    def get_fresh_sonars(self):

        return {
            "left_far":
                self.left_far
                if self.sensor_is_fresh(
                    self.last_left_far_time
                )
                else float("inf"),

            "left_middle":
                self.left_middle
                if self.sensor_is_fresh(
                    self.last_left_middle_time
                )
                else float("inf"),

            "right_far":
                self.right_far
                if self.sensor_is_fresh(
                    self.last_right_far_time
                )
                else float("inf"),

            "right_middle":
                self.right_middle
                if self.sensor_is_fresh(
                    self.last_right_middle_time
                )
                else float("inf"),
        }

    # =========================================================
    # NAV2 HANDOVER DISTANCE
    # =========================================================

    def get_nav2_handover_distance(self):

        speed = abs(
            self.vehicle_speed
        )

        distance = (
            speed
            *
            self.nav2_handover_time
        )

        distance += (
            self.nav2_handover_margin
        )

        distance = max(
            distance,
            self.nav2_handover_min_distance
        )

        distance = min(
            distance,
            self.nav2_handover_max_distance
        )

        return distance

    # =========================================================
    # EMERGENCY BRAKE DISTANCE
    # =========================================================

    def get_emergency_brake_distance(self):

        speed = abs(
            self.vehicle_speed
        )

        reaction_distance = (
            speed
            *
            self.brake_reaction_time
        )

        braking_distance = 0.0

        if self.brake_deceleration > 0.0:

            braking_distance = (
                speed
                *
                speed
            ) / (
                2.0
                *
                self.brake_deceleration
            )

        distance = (
            reaction_distance
            +
            braking_distance
            +
            self.brake_margin
        )

        distance = max(
            distance,
            self.minimum_brake_distance
        )

        distance = min(
            distance,
            self.maximum_brake_distance
        )

        return distance

    # =========================================================
    # LiDAR DETECTION
    # =========================================================

    def lidar_detected(
        self,
        handover_distance
    ):

        if not self.sensor_is_fresh(
            self.last_lidar_time
        ):

            return False

        if not math.isfinite(
            self.lidar_distance
        ):

            return False

        return (
            self.lidar_distance
            <=
            handover_distance
        )

    # =========================================================
    # SONAR STATIC-MAP FILTER
    # =========================================================

    def sonar_is_static_map_match(
        self,
        name,
        distance,
    ):

        if not math.isfinite(distance):
            return False

        frame = self.sonar_frame_ids.get(
            name,
            "",
        )

        map_point = self.transform_point_to_map(
            frame,
            distance,
            0.0,
            0.0,
        )

        if map_point is None:
            # Do not suppress a sonar merely because TF is missing.
            # A missing transform is treated as a real detection
            # candidate until the complete map/TF chain is available.
            return False

        return self.point_is_static_map_geometry(
            map_point[0],
            map_point[1],
        )

    def get_obstacle_sonars(self, sonars):

        filtered = {}

        for name, distance in sonars.items():

            if self.sonar_is_static_map_match(
                name,
                distance,
            ):

                filtered[name] = float("inf")

            else:

                filtered[name] = distance

        return filtered

    # =========================================================
    # SONAR DETECTION
    #
    # EVERY sonar keeps equal priority.
    # A sonar is ignored only when its measured endpoint lies
    # on static geometry already present in /map.
    # =========================================================

    def sonar_detections(
        self,
        sonars,
        handover_distance,
    ):

        detections = []

        sonar_distance_limit = min(
            handover_distance,
            self.sonar_max_detection_distance,
        )

        obstacle_sonars = self.get_obstacle_sonars(
            sonars
        )

        for name, distance in obstacle_sonars.items():

            if (
                math.isfinite(distance)
                and
                distance <= sonar_distance_limit
            ):

                detections.append(
                    (
                        name,
                        distance,
                    )
                )

        return detections

    # =========================================================
    # FUSED DISTANCE
    # =========================================================

    def fused_distance(
        self,
        sonars
    ):

        sonars = self.get_obstacle_sonars(sonars)

        distances = []

        if (
            self.sensor_is_fresh(
                self.last_lidar_time
            )
            and
            math.isfinite(
                self.lidar_distance
            )
        ):

            distances.append(
                self.lidar_distance
            )

        for value in sonars.values():

            if math.isfinite(value):

                distances.append(
                    value
                )

        if not distances:

            return float("inf")

        return min(
            distances
        )

    # =========================================================
    # EMERGENCY BRAKE
    # =========================================================

    def emergency_required(
        self,
        sonars
    ):

        sonars = self.get_obstacle_sonars(sonars)

        brake_distance = (
            self.get_emergency_brake_distance()
        )

        # LiDAR has equal emergency authority.
        lidar_emergency = (
            self.sensor_is_fresh(
                self.last_lidar_time
            )
            and
            math.isfinite(
                self.lidar_distance
            )
            and
            self.lidar_distance
            <=
            brake_distance
        )

        # Every sonar has equal emergency authority.
        sonar_emergency = any(
            (
                math.isfinite(distance)
                and
                distance
                <=
                brake_distance
            )
            for distance in sonars.values()
        )

        return (
            lidar_emergency
            or
            sonar_emergency
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        now = time.monotonic()

        # =====================================================
        # STATIC MAP REQUIRED
        # =====================================================

        if not self.map_is_ready():

            distance_msg = Float64()
            distance_msg.data = 30.0
            self.distance_pub.publish(distance_msg)

            detected_msg = Bool()
            detected_msg.data = False
            self.detected_pub.publish(detected_msg)

            brake_msg = Float64()
            brake_msg.data = 0.0
            self.brake_pub.publish(brake_msg)

            if self.last_status != "WAITING_FOR_MAP":
                self.get_logger().warn(
                    "WAITING_FOR_MAP -> obstacle detector paused"
                )
                self.last_status = "WAITING_FOR_MAP"

            return

        # =====================================================
        # SENSOR DATA
        # =====================================================

        sonars = (
            self.get_fresh_sonars()
        )

        # =====================================================
        # DYNAMIC HANDOVER
        # =====================================================

        handover_distance = (
            self.get_nav2_handover_distance()
        )

        # =====================================================
        # LiDAR
        # =====================================================

        lidar_seen = (
            self.lidar_detected(
                handover_distance
            )
        )

        # =====================================================
        # SONARS
        # =====================================================

        sonar_hits = (
            self.sonar_detections(
                sonars,
                handover_distance
            )
        )

        sonar_seen = (
            len(sonar_hits)
            > 0
        )

        # =====================================================
        # EQUAL PRIORITY
        # =====================================================

        obstacle_seen = (
            lidar_seen
            or
            sonar_seen
        )

        # =====================================================
        # DISTANCE
        # =====================================================

        distance = (
            self.fused_distance(
                sonars
            )
        )

        # =====================================================
        # DETECTION SOURCE
        # =====================================================

        sources = []

        if lidar_seen:

            sources.append(
                "LIDAR"
            )

        for name, _ in sonar_hits:

            sources.append(
                name.upper()
            )

        current_source = (
            "+".join(sources)
            if sources
            else
            "NONE"
        )

        # =====================================================
        # TEMPORAL CONFIRMATION
        # =====================================================

        if obstacle_seen:

            self.detect_count += 1

            self.clear_count = 0

            self.last_obstacle_seen_time = now

        else:

            self.detect_count = 0

            if self.obstacle_active:

                self.clear_count += 1

            else:

                self.clear_count = 0

        # =====================================================
        # CONFIRM
        # =====================================================

        if (
            not self.obstacle_active
            and
            self.detect_count
            >=
            self.confirmation_cycles
        ):

            self.obstacle_active = True

            self.clear_count = 0

            self.last_detection_source = (
                current_source
            )

            self.get_logger().warn(
                "OBSTACLE CONFIRMED | "
                "SOURCE=%s | "
                "DIST=%.2f m | "
                "NAV2 HANDOVER=%.2f m"
                %
                (
                    current_source,

                    distance
                    if math.isfinite(
                        distance
                    )
                    else -1.0,

                    handover_distance
                )
            )

        # =====================================================
        # HOLD
        # =====================================================

        if self.obstacle_active:

            time_since_seen = (
                now
                -
                self.last_obstacle_seen_time
            )

            if (
                time_since_seen
                <=
                self.obstacle_hold_time
            ):

                self.clear_count = 0

            elif self.clear_count >= self.clear_cycles:

                self.obstacle_active = False

                self.clear_count = 0

                self.last_detection_source = ""

                self.get_logger().info(
                    "OBSTACLE CLEARED"
                )

        # =====================================================
        # EMERGENCY BRAKE
        # =====================================================

        emergency = (
            self.emergency_required(
                sonars
            )
        )

        if emergency:

            self.brake_until = (
                now
                +
                self.brake_hold_time
            )

        brake_latched = (
            now
            <
            self.brake_until
        )

        brake = (
            1.0
            if brake_latched
            else 0.0
        )

        # =====================================================
        # STATUS
        # =====================================================

        if brake_latched:

            status = "EMERGENCY"

        elif self.obstacle_active:

            status = "NAV2_TRIGGER"

        elif obstacle_seen:

            status = "DETECTING"

        else:

            status = "CLEAR"

        # =====================================================
        # PUBLISH DISTANCE
        # =====================================================

        distance_msg = Float64()

        distance_msg.data = (
            float(distance)
            if math.isfinite(
                distance
            )
            else 30.0
        )

        self.distance_pub.publish(
            distance_msg
        )

        # =====================================================
        # PUBLISH OBSTACLE
        # =====================================================

        detected_msg = Bool()

        detected_msg.data = bool(
            self.obstacle_active
        )

        self.detected_pub.publish(
            detected_msg
        )

        # =====================================================
        # PUBLISH BRAKE
        # =====================================================

        brake_msg = Float64()

        brake_msg.data = float(
            brake
        )

        self.brake_pub.publish(
            brake_msg
        )

        # =====================================================
        # STATUS LOG
        # =====================================================

        if status != self.last_status:

            self.get_logger().info(
                "STATUS=%s | "
                "SOURCE=%s | "
                "LiDAR=%.2f m | "
                "STATIC_MAP=%.2f m | "
                "FUSED=%.2f m | "
                "SPEED=%.2f m/s | "
                "HANDOVER=%.2f m | "
                "BRAKE=%.2f m"
                %
                (
                    status,

                    current_source,

                    self.lidar_distance
                    if math.isfinite(
                        self.lidar_distance
                    )
                    else -1.0,

                    self.lidar_static_distance
                    if math.isfinite(
                        self.lidar_static_distance
                    )
                    else -1.0,

                    distance
                    if math.isfinite(
                        distance
                    )
                    else -1.0,

                    self.vehicle_speed,

                    handover_distance,

                    self.get_emergency_brake_distance()
                )
            )

            self.last_status = status

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def shutdown(self):

        msg = Float64()

        msg.data = 1.0

        for _ in range(5):

            self.brake_pub.publish(
                msg
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ObstacleDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()