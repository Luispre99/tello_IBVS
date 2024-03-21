#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from pynput import keyboard
from geometry_msgs.msg import Twist

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pressed_keys = set()
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if not self.pressed_keys:
            self.vel_x = 0.0
            self.vel_y = 0.0
        self.publish_velocity()

    def on_press(self, key):
        c = 0.2
        if key in {keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right}:
            self.pressed_keys.add(key)
            if keyboard.Key.up in self.pressed_keys:
                self.vel_x = c
            if keyboard.Key.down in self.pressed_keys:
                self.vel_x = -c
            if keyboard.Key.left in self.pressed_keys:
                self.vel_y = -c
            if keyboard.Key.right in self.pressed_keys:
                self.vel_y = c

            self.vel_x = np.clip(self.vel_x, -0.2, 0.2)
            self.vel_y = np.clip(self.vel_y, -0.2, 0.2)

            self.publish_velocity()

    def on_release(self, key):
        try:
            self.pressed_keys.remove(key)
        except KeyError:
            pass  # Key was not in the set, ignore.

    def publish_velocity(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.vel_x
        twist_msg.linear.y = self.vel_y
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    key_publisher = KeyPublisher()
    rclpy.spin(key_publisher)

if __name__ == '__main__':
    main()

