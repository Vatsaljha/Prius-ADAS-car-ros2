#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node

from prius_msgs.msg import Control


class KeyboardTeleop(Node):

    def __init__(self):

        super().__init__('keyboard_teleop')

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.control_pub = self.create_publisher(
            Control,
            '/prius/control',
            10
        )

        # ============================================================
        # BASIC SETTINGS
        # ============================================================

        # Fixed throttle.
        #
        # W always commands this throttle.
        # It does NOT automatically increase.
        #
        self.forward_throttle = 0.18

        # Same basic throttle for reverse
        self.reverse_throttle = 0.15

        # ============================================================
        # VEHICLE STATE
        # ============================================================

        self.throttle = 0.0
        self.brake = 0.0

        self.steer = 0.0

        self.gear = Control.NEUTRAL

        # ============================================================
        # DRIVING STATE
        # ============================================================

        self.forward_active = False
        self.reverse_active = False

        # ============================================================
        # STEERING
        # ============================================================

        self.max_steer = 1.0

        # How quickly steering reaches full lock
        self.steer_rate = 7.0

        # How quickly steering returns to center
        self.center_rate = 5.0

        # Keyboard timeout.
        #
        # Since terminal input does not provide a reliable key-release
        # event, steering is considered released when no A/D event
        # arrives for this amount of time.
        #
        self.steer_timeout = 0.10

        self.steer_direction = 0

        self.last_steer_input = 0.0

        # ============================================================
        # CONTROL LOOP
        # ============================================================

        self.control_period = 0.02

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop
        )

        self.get_logger().info(
            'Clean manual Prius teleop started.'
        )

    # ================================================================
    # STEERING
    # ================================================================

    def steer_right(self):

        self.steer_direction = 1

        self.last_steer_input = time.monotonic()

    # ----------------------------------------------------------------

    def steer_left(self):

        self.steer_direction = -1

        self.last_steer_input = time.monotonic()

    # ----------------------------------------------------------------

    def center_steering(self):

        self.steer_direction = 0

        self.steer = 0.0

    # ================================================================
    # AUTOMATIC STEERING CENTER
    # ================================================================

    def update_steering(self):

        now = time.monotonic()

        # A/D input is still considered active
        active = (
            self.steer_direction != 0
            and
            (
                now - self.last_steer_input
            ) < self.steer_timeout
        )

        # ============================================================
        # A/D ACTIVE
        # ============================================================

        if active:

            target = (
                self.steer_direction *
                self.max_steer
            )

            step = (
                self.steer_rate *
                self.control_period
            )

            if self.steer < target:

                self.steer = min(
                    target,
                    self.steer + step
                )

            elif self.steer > target:

                self.steer = max(
                    target,
                    self.steer - step
                )

        # ============================================================
        # A/D RELEASED
        # ============================================================

        else:

            self.steer_direction = 0

            step = (
                self.center_rate *
                self.control_period
            )

            # Return toward zero
            if self.steer > 0.0:

                self.steer = max(
                    0.0,
                    self.steer - step
                )

            elif self.steer < 0.0:

                self.steer = min(
                    0.0,
                    self.steer + step
                )

            # Remove tiny residual value
            if abs(self.steer) < 0.01:

                self.steer = 0.0

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # Steering
        # ------------------------------------------------------------

        self.update_steering()

        # ------------------------------------------------------------
        # Throttle
        # ------------------------------------------------------------

        if self.forward_active:

            self.throttle = (
                self.forward_throttle
            )

            self.brake = 0.0

            self.gear = Control.FORWARD

        elif self.reverse_active:

            self.throttle = (
                self.reverse_throttle
            )

            self.brake = 0.0

            self.gear = Control.REVERSE

        else:

            self.throttle = 0.0

            self.brake = 0.0

            self.gear = Control.NEUTRAL

        # ------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------

        msg = Control()

        msg.throttle = float(
            self.throttle
        )

        msg.brake = float(
            self.brake
        )

        msg.steer = float(
            self.steer
        )

        msg.shift_gears = self.gear

        self.control_pub.publish(
            msg
        )

    # ================================================================
    # FORWARD
    # ================================================================

    def forward(self):

        self.forward_active = True

        self.reverse_active = False

        self.brake = 0.0

        self.gear = Control.FORWARD

        print(
            "\rFORWARD        ",
            end="",
            flush=True
        )

    # ================================================================
    # REVERSE
    # ================================================================

    def reverse(self):

        self.forward_active = False

        self.reverse_active = True

        self.brake = 0.0

        self.gear = Control.REVERSE

        print(
            "\rREVERSE        ",
            end="",
            flush=True
        )

    # ================================================================
    # BRAKE
    # ================================================================

    def brake_vehicle(self):

        self.forward_active = False

        self.reverse_active = False

        self.throttle = 0.0

        self.brake = 1.0

        self.gear = Control.NEUTRAL

        self.center_steering()

        print(
            "\rBRAKE          ",
            end="",
            flush=True
        )

    # ================================================================
    # STOP
    # ================================================================

    def stop_vehicle(self):

        self.forward_active = False

        self.reverse_active = False

        self.throttle = 0.0

        self.brake = 0.0

        self.gear = Control.NEUTRAL

    # ================================================================
    # KEYBOARD
    # ================================================================

    def keyboard_loop(self):

        print()
        print("=" * 58)
        print("             PRIUS BASIC MANUAL TELEOP")
        print("=" * 58)
        print()
        print(" W       = FORWARD")
        print(" S       = REVERSE")
        print(" A       = RIGHT")
        print(" D       = LEFT")
        print(" Q       = CENTER STEERING")
        print(" SPACE   = BRAKE")
        print(" X       = EXIT")
        print()
        print("Steering automatically returns to CENTER")
        print("when A/D input stops.")
        print()
        print(
            f"Fixed forward throttle: "
            f"{self.forward_throttle:.2f}"
        )
        print(
            f"Fixed reverse throttle: "
            f"{self.reverse_throttle:.2f}"
        )
        print()
        print("No speed modes.")
        print("No automatic acceleration.")
        print("No dashboard connection.")
        print("=" * 58)
        print()

        old_settings = termios.tcgetattr(
            sys.stdin
        )

        try:

            tty.setcbreak(
                sys.stdin.fileno()
            )

            while rclpy.ok():

                ready, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    self.control_period
                )

                if ready:

                    key = sys.stdin.read(
                        1
                    ).lower()

                    # ----------------------------------------------
                    # FORWARD
                    # ----------------------------------------------

                    if key == 'w':

                        self.forward()

                    # ----------------------------------------------
                    # REVERSE
                    # ----------------------------------------------

                    elif key == 's':

                        self.reverse()

                    # ----------------------------------------------
                    # RIGHT
                    # ----------------------------------------------

                    elif key == 'a':

                        self.steer_right()

                    # ----------------------------------------------
                    # LEFT
                    # ----------------------------------------------

                    elif key == 'd':

                        self.steer_left()

                    # ----------------------------------------------
                    # CENTER
                    # ----------------------------------------------

                    elif key == 'q':

                        self.center_steering()

                        print(
                            "\rSTEERING CENTER     ",
                            end="",
                            flush=True
                        )

                    # ----------------------------------------------
                    # BRAKE
                    # ----------------------------------------------

                    elif key == ' ':

                        self.brake_vehicle()

                    # ----------------------------------------------
                    # EXIT
                    # ----------------------------------------------

                    elif key == 'x':

                        self.brake_vehicle()

                        time.sleep(
                            0.3
                        )

                        break

                # --------------------------------------------------
                # ROS callbacks
                # --------------------------------------------------

                rclpy.spin_once(
                    self,
                    timeout_sec=0.0
                )

        finally:

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

            # ========================================================
            # SAFETY STOP
            # ========================================================

            self.forward_active = False

            self.reverse_active = False

            self.throttle = 0.0

            self.brake = 1.0

            self.steer = 0.0

            self.gear = Control.NEUTRAL

            for _ in range(10):

                msg = Control()

                msg.throttle = 0.0

                msg.brake = 1.0

                msg.steer = 0.0

                msg.shift_gears = (
                    Control.NEUTRAL
                )

                self.control_pub.publish(
                    msg
                )

                rclpy.spin_once(
                    self,
                    timeout_sec=0.01
                )

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        msg = Control()

        msg.throttle = 0.0

        msg.brake = 1.0

        msg.steer = 0.0

        msg.shift_gears = Control.NEUTRAL

        for _ in range(5):

            self.control_pub.publish(
                msg
            )

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = KeyboardTeleop()

    try:

        node.keyboard_loop()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()