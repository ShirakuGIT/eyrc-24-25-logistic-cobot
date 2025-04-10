#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
"""

import math
import time
from copy import deepcopy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

linear_speed = 0.2  # Set your desired linear speed
angular_speed = 0.0  # Set your desired angular speed

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    
    def servo_circular_motion():
        """Publish a TwistStamped message to move in a specific direction."""
        twist_msg = TwistStamped()
        
        # Set the current timestamp and frame ID for the header
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = ur5.base_link_name()  # or any other frame ID you need

        # Set linear speeds for the circular motion example
        twist_msg.twist.linear.x = linear_speed 
        twist_msg.twist.linear.y = linear_speed 
        twist_msg.twist.linear.z = 0.0
        
        # Set angular speeds (adjust as needed)
        twist_msg.twist.angular.x = angular_speed
        twist_msg.twist.angular.y = angular_speed
        twist_msg.twist.angular.z = angular_speed
        
        # Publish the twist message
        twist_pub.publish(twist_msg)

    # Create a timer to call `servo_circular_motion` at 50 Hz (adjustable)
    node.create_timer(0.02, servo_circular_motion)  # 0.02s interval for 50 Hz

    # Spin the node to keep it alive
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
