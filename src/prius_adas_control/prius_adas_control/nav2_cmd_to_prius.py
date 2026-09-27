#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from prius_msgs.msg import Control
from rclpy.node import Node


class Nav2CmdToPrius(Node):
    """Convert Nav2 Twist commands into the Prius control message.

    Final chain:
        /cmd_vel -> /prius/control

    The adapter enforces the physical Ackermann steering limit before
    converting yaw rate into normalized Prius steering.
    """

    def __init__(self):
        super().__init__("nav2_cmd_to_prius")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_topic", "/prius/control")

        self.declare_parameter("wheelbase", 2.70)
        self.declare_parameter("max_steering_angle", 0.55)
        self.declare_parameter("max_speed", 0.25)
        self.declare_parameter("max_throttle", 0.06)
        self.declare_parameter("minimum_throttle", 0.020)
        self.declare_parameter("command_timeout", 0.75)
        self.declare_parameter("steering_sign", -1.0)

        self.declare_parameter("steering_alpha", 0.25)
        self.declare_parameter("max_steering_rate", 1.10)
        self.declare_parameter("steering_deadband", 0.015)

        self.declare_parameter("startup_duration", 2.0)
        self.declare_parameter("startup_speed", 0.055)
        self.declare_parameter("startup_steering_limit", 0.35)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)

        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steering_angle = float(
            self.get_parameter("max_steering_angle").value
        )
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_throttle = float(self.get_parameter("max_throttle").value)
        self.minimum_throttle = float(
            self.get_parameter("minimum_throttle").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.steering_sign = float(self.get_parameter("steering_sign").value)

        self.steering_alpha = float(
            self.get_parameter("steering_alpha").value
        )
        self.max_steering_rate = float(
            self.get_parameter("max_steering_rate").value
        )
        self.steering_deadband = float(
            self.get_parameter("steering_deadband").value
        )

        self.startup_duration = float(
            self.get_parameter("startup_duration").value
        )
        self.startup_speed = float(
            self.get_parameter("startup_speed").value
        )
        self.startup_steering_limit = float(
            self.get_parameter("startup_steering_limit").value
        )

        self.control_pub = self.create_publisher(Control, self.control_topic, 10)
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_update_time = time.monotonic()
        self.start_time = time.monotonic()
        self.last_log_time = 0.0
        self.current_steering = 0.0

        self.create_timer(0.05, self.update)

        self.get_logger().info("Nav2 -> Prius Ackermann adapter v3 started")
        self.get_logger().info(
            f"input={self.cmd_vel_topic} output={self.control_topic} "
            f"L={self.wheelbase:.2f} max_delta={self.max_steering_angle:.2f} "
            f"sign={self.steering_sign:+.0f}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.monotonic()
        self.last_linear = float(msg.linear.x)
        self.last_angular = float(msg.angular.z)

        if not math.isfinite(self.last_linear):
            self.last_linear = 0.0
        if not math.isfinite(self.last_angular):
            self.last_angular = 0.0

    def update(self):
        now = time.monotonic()
        dt = max(0.001, min(0.10, now - self.last_update_time))
        self.last_update_time = now

        if now - self.last_cmd_time > self.command_timeout:
            self.return_steering(dt)
            self.publish_stop()
            return

        linear = max(0.0, min(self.max_speed, self.last_linear))
        angular = self.last_angular

        if linear <= 0.01:
            self.return_steering(dt)
            self.publish_stop()
            return

        # The Prius cannot generate arbitrary yaw rate at a given speed.
        physical_max_angular = (
            linear * math.tan(self.max_steering_angle) / self.wheelbase
        )
        angular = max(
            -physical_max_angular,
            min(physical_max_angular, angular),
        )

        steering_angle = math.atan2(
            self.wheelbase * angular,
            max(linear, 0.02),
        )
        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_angle),
        )

        target_steering = (
            self.steering_sign
            * steering_angle
            / self.max_steering_angle
        )
        target_steering = max(-1.0, min(1.0, target_steering))

        if abs(target_steering) < self.steering_deadband:
            target_steering = 0.0

        # Startup protection reduces the chance of a hard initial steering
        # transient before the localization/controller state settles.
        startup = now - self.start_time < self.startup_duration
        if startup:
            linear = min(linear, self.startup_speed)
            target_steering = max(
                -self.startup_steering_limit,
                min(self.startup_steering_limit, target_steering),
            )

        filtered = (
            self.steering_alpha * target_steering
            + (1.0 - self.steering_alpha) * self.current_steering
        )

        maximum_change = self.max_steering_rate * dt
        difference = max(
            -maximum_change,
            min(maximum_change, filtered - self.current_steering),
        )
        self.current_steering += difference
        self.current_steering = max(-1.0, min(1.0, self.current_steering))

        speed_ratio = max(0.0, min(1.0, linear / self.max_speed))
        throttle = self.minimum_throttle + speed_ratio * (
            self.max_throttle - self.minimum_throttle
        )
        throttle = max(0.0, min(self.max_throttle, throttle))

        output = Control()
        output.throttle = float(throttle)
        output.brake = 0.0
        output.steer = float(self.current_steering)
        output.shift_gears = Control.FORWARD
        self.control_pub.publish(output)

        if now - self.last_log_time >= 0.5:
            self.get_logger().info(
                f"NAV2->PRIUS: v={linear:.3f} "
                f"w={angular:.3f} steer={self.current_steering:.3f} "
                f"thr={throttle:.4f} startup={startup}"
            )
            self.last_log_time = now

    def return_steering(self, dt):
        maximum_change = self.max_steering_rate * dt
        if self.current_steering > 0.0:
            self.current_steering = max(
                0.0, self.current_steering - maximum_change
            )
        elif self.current_steering < 0.0:
            self.current_steering = min(
                0.0, self.current_steering + maximum_change
            )

    def publish_stop(self):
        msg = Control()
        msg.throttle = 0.0
        msg.brake = 1.0
        msg.steer = float(self.current_steering)
        msg.shift_gears = Control.NEUTRAL
        self.control_pub.publish(msg)

    def destroy_node(self):
        if rclpy.ok():
            try:
                self.publish_stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdToPrius()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
