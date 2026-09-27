#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from rclpy.duration import Duration

from prius_msgs.msg import Control
from sensor_msgs.msg import Joy

STEERING_AXIS = 0
THROTTLE_AXIS = 4


class Translator(Node):
    def __init__(self, node_name: str = "joy_translator",
                 namespace: str = "prius") -> None:
        super().__init__(node_name=node_name, namespace=namespace)
        qos = QoSReliabilityPolicy(1)
        self.pub = self.create_publisher(Control, "control", qos_profile=qos)
        self.sub = self.create_subscription(Joy, "joy", self.joy_cb)
        self.last_published_time = self.get_clock().now()
        self.last_published = None
        self.timer = self.create_timer(1./20., self.timer_cb)

    def timer_cb(self):
        if self.last_published and self.last_published_time < \
                self.get_clock().now() + Duration(1.0/20.):
            self.joy_cb(self.last_published)

    def joy_cb(self, msg: Joy):
        self.get_logger().debug(f"{self.get_name()} received axes {msg.axes}")
        command = Control()
        command.header = msg.header
        if msg.axes[THROTTLE_AXIS] >= 0:
            command.throttle = msg.axes[THROTTLE_AXIS]
            command.brake = 0.0
        else:
            command.brake = msg.axes[THROTTLE_AXIS] * -1
            command.throttle = 0.0

        if msg.buttons[3]:
            command.shift_gears = Control.FORWARD
        elif msg.buttons[1]:
            command.shift_gears = Control.NEUTRAL
        elif msg.buttons[0]:
            command.shift_gears = Control.REVERSE
        else:
            command.shift_gears = Control.NO_COMMAND

        command.steer = msg.axes[STEERING_AXIS]
        self.pub.publish(command)