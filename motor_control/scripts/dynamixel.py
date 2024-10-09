#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Bool

class Dynamixel(Node):
    def __init__(self) -> None:
        super().__init__("Dynamixel")
        self.dynamixel_pub = self.create_publisher(Int64, "/dynamixel", 10)
        self.dynamixel_timer = self.create_timer(0.1, self.dynamixel_callback)
    
    def dynamixel_callback(self) -> None:
        msg = Int64()
        msg.data = 10
        self.dynamixel_pub.publish(msg)
    
if __name__ == "__main__":
    rclpy.init()
    node = Dynamixel()
    rclpy.spin(node)
    rclpy.shutdown()
    