#!/usr/bin/env python3

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



class LaneDetection(Node):

    def __init__(self):

        super().__init__("lane_detection")

        self.bridge = CvBridge()

        # =====================================================
        # CAMERA
        # =====================================================

        self.image_sub = self.create_subscription(
            Image,
            "/prius/front_camera/image_raw",
            self.image_callback,
            10,
        )

        # =====================================================
        # LANE OUTPUTS
        # =====================================================

        self.debug_pub = self.create_publisher(
            Image,
            "/adas/lane_debug",
            10,
        )

        self.lateral_pub = self.create_publisher(
            Float64,
            "/adas/lane_lateral_error",
            10,
        )

        self.heading_pub = self.create_publisher(
            Float64,
            "/adas/lane_heading_error",
            10,
        )

        self.lane_valid_pub = self.create_publisher(
            Bool,
            "/adas/lane_valid",
            10,
        )

        # =====================================================
        # OBSTACLE SAFETY INPUTS
        # =====================================================

        self.obstacle_distance_sub = self.create_subscription(
            Float64,
            "/adas/obstacle_distance",
            self.obstacle_distance_callback,
            10,
        )

        self.brake_sub = self.create_subscription(
            Float64,
            "/adas/brake",
            self.brake_callback,
            10,
        )

        # =====================================================
        # OBSTACLE STATE
        # =====================================================

        # When no obstacle detector data has arrived yet,
        # assume a large distance for display purposes only.
        self.obstacle_distance = 30.0

        # 0.0 = no brake
        # 1.0 = brake
        self.brake_status = 0.0

        # =====================================================
        # FILTERING
        # =====================================================

        self.alpha = 0.15

        self.last_lateral = 0.0
        self.last_heading = 0.0

        # Previous valid curves.
        self.last_yellow_fit = None
        self.last_left_white_fit = None

        self.yellow_lost_frames = 0
        self.white_lost_frames = 0

        self.max_hold_frames = 8

        # =====================================================
        # CAMERA REFERENCE
        # =====================================================

        self.vehicle_center_ratio = 0.50

        # =====================================================
        # REGION OF INTEREST
        # =====================================================

        self.roi_top_ratio = 0.45

        # =====================================================
        # YELLOW HSV
        # =====================================================

        self.yellow_lower = np.array(
            [10, 70, 70],
            dtype=np.uint8,
        )

        self.yellow_upper = np.array(
            [45, 255, 255],
            dtype=np.uint8,
        )

        # =====================================================
        # WHITE HSV
        # =====================================================

        self.white_lower = np.array(
            [0, 0, 160],
            dtype=np.uint8,
        )

        self.white_upper = np.array(
            [180, 85, 255],
            dtype=np.uint8,
        )

        # =====================================================
        # CURVE FIT
        # =====================================================

        self.minimum_points = 25

        # =====================================================
        # LOOKAHEAD
        # =====================================================

        self.y_bottom_ratio = 0.96
        self.y_lookahead_ratio = 0.68

        # =====================================================
        # ERROR LIMITS
        # =====================================================

        self.max_lateral_error = 0.70
        self.max_heading_error = 0.90

        # =====================================================
        # FALLBACK LANE WIDTH
        # =====================================================

        self.yellow_only_bottom_offset = 0.28
        self.yellow_only_lookahead_offset = 0.17

        self.white_only_bottom_offset = 0.28
        self.white_only_lookahead_offset = 0.17

        # =====================================================
        # LOGGING
        # =====================================================

        self.frame_counter = 0

        self.get_logger().info(
            "Lane detector initialized: "
            "LEFT WHITE EDGE + YELLOW CENTERLINE"
        )

    # =========================================================
    # OBSTACLE DISTANCE CALLBACK
    # =========================================================

    def obstacle_distance_callback(
        self,
        msg,
    ):

        distance = float(
            msg.data
        )

        if not math.isfinite(distance):

            distance = 30.0

        self.obstacle_distance = max(
            0.0,
            min(
                30.0,
                distance,
            ),
        )

    # =========================================================
    # BRAKE CALLBACK
    # =========================================================

    def brake_callback(
        self,
        msg,
    ):

        brake = float(
            msg.data
        )

        if not math.isfinite(brake):

            brake = 0.0

        self.brake_status = max(
            0.0,
            min(
                1.0,
                brake,
            ),
        )

    # =========================================================
    # IMAGE CALLBACK
    # =========================================================

    def image_callback(
        self,
        msg,
    ):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as exc:

            self.get_logger().error(
                f"Image conversion failed: {exc}"
            )

            return

        debug = self.process_image(
            frame
        )

        try:

            debug_msg = self.bridge.cv2_to_imgmsg(
                debug,
                encoding="bgr8",
            )

            debug_msg.header = msg.header

            self.debug_pub.publish(
                debug_msg
            )

        except Exception as exc:

            self.get_logger().error(
                f"Debug image publish failed: {exc}"
            )

    # =========================================================
    # PROCESS IMAGE
    # =========================================================

    def process_image(
        self,
        frame,
    ):

        self.frame_counter += 1

        height, width = frame.shape[:2]

        # =====================================================
        # 1. GAUSSIAN BLUR
        # =====================================================

        blurred = cv2.GaussianBlur(
            frame,
            (5, 5),
            0,
        )

        # =====================================================
        # 2. HSV
        # =====================================================

        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV,
        )

        # =====================================================
        # 3. COLOR SEGMENTATION
        # =====================================================

        yellow_mask = cv2.inRange(
            hsv,
            self.yellow_lower,
            self.yellow_upper,
        )

        white_mask = cv2.inRange(
            hsv,
            self.white_lower,
            self.white_upper,
        )

        # =====================================================
        # 4. ROI
        # =====================================================

        roi = np.zeros(
            (height, width),
            dtype=np.uint8,
        )

        polygon = np.array(
            [[
                (0, height),
                (width, height),
                (
                    int(width * 0.93),
                    int(height * self.roi_top_ratio),
                ),
                (
                    int(width * 0.07),
                    int(height * self.roi_top_ratio),
                ),
            ]],
            dtype=np.int32,
        )

        cv2.fillPoly(
            roi,
            polygon,
            255,
        )

        yellow_mask = cv2.bitwise_and(
            yellow_mask,
            roi,
        )

        white_mask = cv2.bitwise_and(
            white_mask,
            roi,
        )

        # =====================================================
        # 5. MORPHOLOGICAL CLEANUP
        # =====================================================

        close_kernel = np.ones(
            (7, 7),
            dtype=np.uint8,
        )

        open_kernel = np.ones(
            (3, 3),
            dtype=np.uint8,
        )

        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=2,
        )

        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=2,
        )

        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_OPEN,
            open_kernel,
            iterations=1,
        )

        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_OPEN,
            open_kernel,
            iterations=1,
        )

        # =====================================================
        # 6. CANNY
        # =====================================================

        yellow_edges = cv2.Canny(
            yellow_mask,
            100,
            250,
        )

        white_edges = cv2.Canny(
            white_mask,
            100,
            250,
        )

        # Kept for the lane-detection pipeline.
        _ = yellow_edges
        _ = white_edges

        # =====================================================
        # 7. YELLOW CENTERLINE
        # =====================================================

        yellow_y, yellow_x = np.where(
            yellow_mask > 0
        )

        yellow_fit = self.fit_lane_curve(
            yellow_y,
            yellow_x,
            width,
            "yellow",
        )

        # =====================================================
        # YELLOW TEMPORAL HOLD
        # =====================================================

        if yellow_fit is not None:

            self.last_yellow_fit = yellow_fit
            self.yellow_lost_frames = 0

        else:

            self.yellow_lost_frames += 1

            if (
                self.last_yellow_fit is not None
                and
                self.yellow_lost_frames
                <=
                self.max_hold_frames
            ):

                yellow_fit = (
                    self.last_yellow_fit
                )

        yellow_valid = (
            yellow_fit is not None
        )

        # =====================================================
        # 8. SELECT LEFT WHITE EDGE
        # =====================================================

        left_white_x = np.array(
            [],
            dtype=np.int32,
        )

        left_white_y = np.array(
            [],
            dtype=np.int32,
        )

        white_y_all, white_x_all = np.where(
            white_mask > 0
        )

        if len(white_x_all) > 0:

            if yellow_fit is not None:

                yellow_at_white = np.polyval(
                    yellow_fit,
                    white_y_all.astype(
                        np.float64
                    ),
                )

                left_of_yellow = (
                    white_x_all
                    <
                    yellow_at_white
                    -
                    width * 0.025
                )

            else:

                left_of_yellow = (
                    white_x_all
                    <
                    int(
                        width * 0.55
                    )
                )

            lower_region = (
                white_y_all
                >
                int(
                    height * 0.48
                )
            )

            valid_white = (
                left_of_yellow
                &
                lower_region
            )

            left_white_x = white_x_all[
                valid_white
            ]

            left_white_y = white_y_all[
                valid_white
            ]

        # =====================================================
        # 9. FIT LEFT WHITE CURVE
        # =====================================================

        left_white_fit = self.fit_lane_curve(
            left_white_y,
            left_white_x,
            width,
            "white",
        )

        # =====================================================
        # LEFT WHITE TEMPORAL HOLD
        # =====================================================

        if left_white_fit is not None:

            self.last_left_white_fit = (
                left_white_fit
            )

            self.white_lost_frames = 0

        else:

            self.white_lost_frames += 1

            if (
                self.last_left_white_fit is not None
                and
                self.white_lost_frames
                <=
                self.max_hold_frames
            ):

                left_white_fit = (
                    self.last_left_white_fit
                )

        white_valid = (
            left_white_fit is not None
        )

        # =====================================================
        # DEBUG IMAGE
        # =====================================================

        debug = frame.copy()

        # =====================================================
        # DRAW ROI
        # =====================================================

        cv2.polylines(
            debug,
            polygon,
            True,
            (255, 255, 255),
            2,
        )

        # =====================================================
        # DRAW YELLOW PIXELS
        # =====================================================

        if len(yellow_x) > 0:

            step = max(
                1,
                len(yellow_x) // 500,
            )

            for x, y in zip(
                yellow_x[::step],
                yellow_y[::step],
            ):

                cv2.circle(
                    debug,
                    (int(x), int(y)),
                    2,
                    (0, 180, 255),
                    -1,
                )

        # =====================================================
        # DRAW WHITE PIXELS
        # =====================================================

        if len(left_white_x) > 0:

            step = max(
                1,
                len(left_white_x) // 500,
            )

            for x, y in zip(
                left_white_x[::step],
                left_white_y[::step],
            ):

                cv2.circle(
                    debug,
                    (int(x), int(y)),
                    2,
                    (255, 180, 0),
                    -1,
                )

        # =====================================================
        # DRAW YELLOW CURVE
        # =====================================================

        if yellow_valid:

            self.draw_curve(
                debug,
                yellow_fit,
                height,
                (0, 255, 255),
                6,
            )

        # =====================================================
        # DRAW WHITE CURVE
        # =====================================================

        if white_valid:

            self.draw_curve(
                debug,
                left_white_fit,
                height,
                (0, 255, 0),
                6,
            )

        # =====================================================
        # REFERENCE POSITIONS
        # =====================================================

        y_bottom = int(
            height
            *
            self.y_bottom_ratio
        )

        y_lookahead = int(
            height
            *
            self.y_lookahead_ratio
        )

        # =====================================================
        # EVALUATE YELLOW
        # =====================================================

        yellow_bottom = None
        yellow_lookahead = None

        if yellow_valid:

            yellow_bottom = (
                self.evaluate_curve(
                    yellow_fit,
                    y_bottom,
                )
            )

            yellow_lookahead = (
                self.evaluate_curve(
                    yellow_fit,
                    y_lookahead,
                )
            )

        # =====================================================
        # EVALUATE WHITE
        # =====================================================

        white_bottom = None
        white_lookahead = None

        if white_valid:

            white_bottom = (
                self.evaluate_curve(
                    left_white_fit,
                    y_bottom,
                )
            )

            white_lookahead = (
                self.evaluate_curve(
                    left_white_fit,
                    y_lookahead,
                )
            )

        # =====================================================
        # VALIDATE X
        # =====================================================

        yellow_bottom = self.valid_x(
            yellow_bottom,
            width,
        )

        yellow_lookahead = self.valid_x(
            yellow_lookahead,
            width,
        )

        white_bottom = self.valid_x(
            white_bottom,
            width,
        )

        white_lookahead = self.valid_x(
            white_lookahead,
            width,
        )

        # =====================================================
        # GEOMETRIC VALIDATION
        #
        # LEFT WHITE < YELLOW
        # =====================================================

        if (
            white_bottom is not None
            and
            yellow_bottom is not None
            and
            white_bottom >= yellow_bottom
        ):

            white_bottom = None

        if (
            white_lookahead is not None
            and
            yellow_lookahead is not None
            and
            white_lookahead >= yellow_lookahead
        ):

            white_lookahead = None

        # =====================================================
        # LANE CENTER
        # =====================================================

        lane_bottom = None
        lane_lookahead = None

        # =====================================================
        # BOTH
        # =====================================================

        if (
            white_bottom is not None
            and
            yellow_bottom is not None
            and
            white_lookahead is not None
            and
            yellow_lookahead is not None
        ):

            lane_bottom = (
                white_bottom
                +
                yellow_bottom
            ) / 2.0

            lane_lookahead = (
                white_lookahead
                +
                yellow_lookahead
            ) / 2.0

            mode = "BOTH"

        # =====================================================
        # YELLOW ONLY
        # =====================================================

        elif (
            yellow_bottom is not None
            and
            yellow_lookahead is not None
        ):

            lane_bottom = (
                yellow_bottom
                -
                width
                *
                self.yellow_only_bottom_offset
            )

            lane_lookahead = (
                yellow_lookahead
                -
                width
                *
                self.yellow_only_lookahead_offset
            )

            mode = "YELLOW"

        # =====================================================
        # WHITE ONLY
        # =====================================================

        elif (
            white_bottom is not None
            and
            white_lookahead is not None
        ):

            lane_bottom = (
                white_bottom
                +
                width
                *
                self.white_only_bottom_offset
            )

            lane_lookahead = (
                white_lookahead
                +
                width
                *
                self.white_only_lookahead_offset
            )

            mode = "WHITE"

        # =====================================================
        # HOLD
        # =====================================================

        else:

            lane_bottom = (
                width
                *
                self.vehicle_center_ratio
                +
                self.last_lateral
                *
                width
                *
                0.5
            )

            lane_lookahead = (
                width
                *
                self.vehicle_center_ratio
            )

            mode = "HOLD"

        # =====================================================
        # LIMIT LANE CENTER
        # =====================================================

        lane_bottom = float(
            np.clip(
                lane_bottom,
                width * 0.05,
                width * 0.95,
            )
        )

        lane_lookahead = float(
            np.clip(
                lane_lookahead,
                width * 0.05,
                width * 0.95,
            )
        )

        # =====================================================
        # LANE HEALTH
        #
        # BOTH / YELLOW / WHITE:
        #     Current lane geometry is available.
        #
        # HOLD:
        #     Previous lane geometry is being reused. Treat
        #     this as a temporary perception loss so the safety
        #     supervisor can hand control to Nav2 when the hold
        #     expires.
        # =====================================================

        lane_is_valid = (
            mode != "HOLD"
            and lane_bottom is not None
            and lane_lookahead is not None
        )

        lane_valid_msg = Bool()
        lane_valid_msg.data = bool(lane_is_valid)

        self.lane_valid_pub.publish(
            lane_valid_msg
        )

        # =====================================================
        # VEHICLE CENTER
        # =====================================================

        vehicle_center = (
            width
            *
            self.vehicle_center_ratio
        )

        # =====================================================
        # LATERAL ERROR
        # =====================================================

        lateral_error = (
            lane_bottom
            -
            vehicle_center
        ) / (
            width * 0.5
        )

        # =====================================================
        # HEADING ERROR
        # =====================================================

        dx = (
            lane_bottom
            -
            lane_lookahead
        )

        dy = (
            y_bottom
            -
            y_lookahead
        )

        if dy <= 0:

            heading_error = 0.0

        else:

            heading_error = math.atan2(
                dx,
                dy,
            )

        # =====================================================
        # LIMIT ERRORS
        # =====================================================

        lateral_error = float(
            np.clip(
                lateral_error,
                -self.max_lateral_error,
                self.max_lateral_error,
            )
        )

        heading_error = float(
            np.clip(
                heading_error,
                -self.max_heading_error,
                self.max_heading_error,
            )
        )

        # =====================================================
        # TEMPORAL FILTER
        # =====================================================

        self.last_lateral = (
            (1.0 - self.alpha)
            *
            self.last_lateral
            +
            self.alpha
            *
            lateral_error
        )

        self.last_heading = (
            (1.0 - self.alpha)
            *
            self.last_heading
            +
            self.alpha
            *
            heading_error
        )

        # =====================================================
        # PUBLISH LATERAL
        # =====================================================

        lateral_msg = Float64()

        lateral_msg.data = float(
            self.last_lateral
        )

        self.lateral_pub.publish(
            lateral_msg
        )

        # =====================================================
        # PUBLISH HEADING
        # =====================================================

        heading_msg = Float64()

        heading_msg.data = float(
            self.last_heading
        )

        self.heading_pub.publish(
            heading_msg
        )

        # =====================================================
        # VEHICLE CENTER
        # =====================================================

        cv2.line(
            debug,
            (
                int(vehicle_center),
                y_bottom,
            ),
            (
                int(vehicle_center),
                y_lookahead,
            ),
            (0, 0, 255),
            2,
        )

        # =====================================================
        # LANE CENTER
        # =====================================================

        cv2.circle(
            debug,
            (
                int(lane_bottom),
                y_bottom,
            ),
            10,
            (255, 0, 0),
            -1,
        )

        cv2.circle(
            debug,
            (
                int(lane_lookahead),
                y_lookahead,
            ),
            9,
            (255, 0, 0),
            -1,
        )

        cv2.line(
            debug,
            (
                int(lane_bottom),
                y_bottom,
            ),
            (
                int(lane_lookahead),
                y_lookahead,
            ),
            (255, 0, 0),
            4,
        )

        # =====================================================
        # LATERAL ERROR LINE
        # =====================================================

        cv2.line(
            debug,
            (
                int(vehicle_center),
                y_bottom - 25,
            ),
            (
                int(lane_bottom),
                y_bottom - 25,
            ),
            (0, 0, 255),
            3,
        )

        # =====================================================
        # LANE TEXT
        # =====================================================

        cv2.putText(
            debug,
            f"Lateral: {self.last_lateral:+.3f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            f"Heading: {self.last_heading:+.3f} rad",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            f"MODE: {mode}",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            (
                "LEFT WHITE: OK"
                if white_valid
                else "LEFT WHITE: LOST"
            ),
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            (
                "YELLOW CENTER: OK"
                if yellow_valid
                else "YELLOW CENTER: LOST"
            ),
            (20, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        # =====================================================
        # OBSTACLE STATUS
        # =====================================================

        if self.brake_status > 0.5:

            obstacle_status = "EMERGENCY"

        elif self.obstacle_distance <= 2.0:

            obstacle_status = "WARNING"

        else:

            obstacle_status = "SAFE"

        # =====================================================
        # MINIMUM OBSTACLE DISTANCE
        # =====================================================

        if (
            self.obstacle_distance >= 29.99
        ):

            distance_text = (
                "MIN DIST: NO CLOSE OBSTACLE"
            )

        else:

            distance_text = (
                f"MIN DIST: "
                f"{self.obstacle_distance:.2f} m"
            )

        cv2.putText(
            debug,
            distance_text,
            (20, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )

        # =====================================================
        # OBSTACLE STATUS
        # =====================================================

        cv2.putText(
            debug,
            f"OBSTACLE: {obstacle_status}",
            (20, 210),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )

        # =====================================================
        # BRAKE STATUS
        # =====================================================

        cv2.putText(
            debug,
            (
                "BRAKE: YES"
                if self.brake_status > 0.5
                else "BRAKE: NO"
            ),
            (20, 240),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )

        # =====================================================
        # LEGEND
        # =====================================================

        cv2.putText(
            debug,
            "BLUE = LANE CENTER",
            (20, height - 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            "RED = VEHICLE CENTER",
            (20, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2,
        )

        return debug

    # =========================================================
    # CURVE FIT
    # =========================================================

    def fit_lane_curve(
        self,
        y,
        x,
        width,
        lane_type,
    ):

        if len(x) < self.minimum_points:

            return None

        x = x.astype(
            np.float64
        )

        y = y.astype(
            np.float64
        )

        # =====================================================
        # VALID IMAGE FILTER
        # =====================================================

        valid = (
            (x >= 0)
            &
            (x < width)
            &
            (y >= 0)
        )

        x = x[valid]
        y = y[valid]

        if len(x) < self.minimum_points:

            return None

        # =====================================================
        # REMOVE EXTREME X OUTLIERS
        # =====================================================

        x_low = np.percentile(
            x,
            2,
        )

        x_high = np.percentile(
            x,
            98,
        )

        keep = (
            (x >= x_low)
            &
            (x <= x_high)
        )

        x = x[keep]
        y = y[keep]

        if len(x) < self.minimum_points:

            return None

        # =====================================================
        # USABLE Y RANGE
        # =====================================================

        y_min = int(
            np.percentile(
                y,
                5,
            )
        )

        y_max = int(
            np.percentile(
                y,
                98,
            )
        )

        if (
            y_max - y_min
            <
            40
        ):

            return None

        # =====================================================
        # ROW BINNING
        # =====================================================

        sample_y = []
        sample_x = []

        bin_size = 8

        for y0 in range(
            y_min,
            y_max + 1,
            bin_size,
        ):

            y1 = y0 + bin_size

            in_bin = (
                (y >= y0)
                &
                (y < y1)
            )

            xs = x[in_bin]

            if len(xs) < 2:

                continue

            representative_x = np.median(
                xs
            )

            sample_y.append(
                (y0 + y1) / 2.0
            )

            sample_x.append(
                representative_x
            )

        if len(sample_x) < 8:

            return None

        sample_y = np.asarray(
            sample_y,
            dtype=np.float64,
        )

        sample_x = np.asarray(
            sample_x,
            dtype=np.float64,
        )

        # =====================================================
        # ROBUST QUADRATIC FIT
        # =====================================================

        keep = np.ones(
            len(sample_x),
            dtype=bool,
        )

        for _ in range(3):

            if np.sum(keep) < 8:

                return None

            try:

                fit = np.polyfit(
                    sample_y[keep],
                    sample_x[keep],
                    2,
                )

            except Exception:

                return None

            predicted = np.polyval(
                fit,
                sample_y,
            )

            residual = np.abs(
                sample_x
                -
                predicted
            )

            threshold = max(
                12.0,
                np.percentile(
                    residual,
                    75,
                )
                *
                1.8,
            )

            new_keep = (
                residual
                <=
                threshold
            )

            if np.array_equal(
                new_keep,
                keep,
            ):

                break

            keep = new_keep

        if np.sum(keep) < 8:

            return None

        try:

            fit = np.polyfit(
                sample_y[keep],
                sample_x[keep],
                2,
            )

        except Exception:

            return None

        # =====================================================
        # SANITY CHECK
        # =====================================================

        test_y = np.array(
            [
                np.min(sample_y),
                np.mean(sample_y),
                np.max(sample_y),
            ],
            dtype=np.float64,
        )

        test_x = np.polyval(
            fit,
            test_y,
        )

        if np.any(
            ~np.isfinite(test_x)
        ):

            return None

        if np.any(
            test_x
            <
            -0.30 * width
        ):

            return None

        if np.any(
            test_x
            >
            1.30 * width
        ):

            return None

        return fit

    # =========================================================
    # EVALUATE CURVE
    # =========================================================

    def evaluate_curve(
        self,
        fit,
        y,
    ):

        return float(
            np.polyval(
                fit,
                y,
            )
        )

    # =========================================================
    # VALIDATE X
    # =========================================================

    def valid_x(
        self,
        value,
        width,
    ):

        if value is None:

            return None

        if not np.isfinite(value):

            return None

        if (
            value
            <
            -0.20 * width
        ):

            return None

        if (
            value
            >
            1.20 * width
        ):

            return None

        return float(value)

    # =========================================================
    # DRAW CURVE
    # =========================================================

    def draw_curve(
        self,
        image,
        fit,
        height,
        color,
        thickness,
    ):

        if fit is None:

            return

        width = image.shape[1]

        points = []

        y_start = int(
            height
            *
            self.roi_top_ratio
        )

        for y in np.linspace(
            y_start,
            height,
            120,
        ):

            x = self.evaluate_curve(
                fit,
                y,
            )

            if (
                -0.5 * width
                <
                x
                <
                1.5 * width
            ):

                points.append(
                    (
                        int(x),
                        int(y),
                    )
                )

        if len(points) < 2:

            return

        for i in range(
            len(points) - 1
        ):

            cv2.line(
                image,
                points[i],
                points[i + 1],
                color,
                thickness,
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = LaneDetection()

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


if __name__ == "__main__":

    main()