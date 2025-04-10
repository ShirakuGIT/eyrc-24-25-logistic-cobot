#!/usr/bin/env python3
"""
Script to navigate the UR5's end effector to specified target frames using MoveIt Servo.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import tf2_ros
from moveit2_servo import MoveIt2Servo
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import math
import tf_transformations
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

class UR5Navigator(Node):
    def __init__(self):
        super().__init__("ex_servo")

        # Create callback group
        self.callback_group = ReentrantCallbackGroup()

        # Create the twist publisher
        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10
        )

        # Set up TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Define a list of object frames to navigate to
        self.target_frames = [f'obj_{i}' for i in range(1, 11)]  # Modify range as needed
        self.current_target_index = 0  # Start with the first target

        # Initialize the MoveIt2Servo interface
        self.servo = MoveIt2Servo(
            node=self,
            frame_id='base_link',
            linear_speed=1.0,  # Adjust speed as necessary
            angular_speed=1.0,
            enable_at_init=True,
            callback_group=self.callback_group
        )

        # Start MoveIt2 Servo if it’s not already started
        self.enable_servo()

        # Timer to check and navigate to transforms periodically
        self.timer = self.create_timer(0.02, self.navigate_to_target)  # 50 Hz

    def enable_servo(self):
        """Call the service to start MoveIt2 servo."""
        start_service = self.create_client(Trigger, '/servo_node/start_servo')

        if not start_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Servo service not available. Exiting.")
            rclpy.shutdown()
            return

        # Call the start_servo service
        req = Trigger.Request()
        future = start_service.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info("MoveIt Servo successfully started.")
        else:
            self.get_logger().error("Failed to start MoveIt Servo.")

    def navigate_to_target(self):
        if self.current_target_index >= len(self.target_frames):
            self.get_logger().info("All targets have been reached.")
            self.stop_movement()
            self.destroy_timer(self.timer)
            return

        target_frame = self.target_frames[self.current_target_index]
        try:
            # Lookup transform from base_link to target frame
            transform = self.tf_buffer.lookup_transform(
                'base_link', target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Extract translation (position) and rotation (orientation as quaternion) from transform
            position = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            )
            target_quat = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )

            # Lookup EEF (wrist_3_link) current position and orientation
            eef_transform = self.tf_buffer.lookup_transform(
                'base_link', 'wrist_3_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
            eef_position = (
                eef_transform.transform.translation.x,
                eef_transform.transform.translation.y,
                eef_transform.transform.translation.z
            )
            eef_quat = (
                eef_transform.transform.rotation.x,
                eef_transform.transform.rotation.y,
                eef_transform.transform.rotation.z,
                eef_transform.transform.rotation.w
            )

            # Calculate position differences
            dx = position[0] - eef_position[0]
            dy = position[1] - eef_position[1]
            dz = position[2] - eef_position[2]

            # Calculate rotational difference as angular velocity
            target_to_eef_quat = tf_transformations.quaternion_multiply(
                tf_transformations.quaternion_inverse(eef_quat), target_quat
            )
            angular_velocities = tf_transformations.euler_from_quaternion(target_to_eef_quat)

            # Create and publish the Twist message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = ur5.base_link_name()
            speed_scale = 0.5  # Scale down speeds as needed to avoid fast motions


            max_angular_speed = 0.2  # radians per second
            angular_velocities = [
                max(min(av * speed_scale, max_angular_speed), -max_angular_speed)
                for av in angular_velocities
            ]

            twist_msg.twist.linear.x = speed_scale * dx
            twist_msg.twist.linear.y = speed_scale * dy
            twist_msg.twist.linear.z = speed_scale * dz
            twist_msg.twist.angular.x = speed_scale * angular_velocities[0]
            twist_msg.twist.angular.y = speed_scale * angular_velocities[1]
            twist_msg.twist.angular.z = speed_scale * angular_velocities[2]

            # Publish the twist message
            self.twist_pub.publish(twist_msg)

            # Log the target frame and its position
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            self.get_logger().info(f"Navigating to '{target_frame}' at position {position}, distance: {distance:.3f}")

            # Check if near target
            if distance < 0.05:
                self.get_logger().info(f"Arrived at '{target_frame}'.")
                self.current_target_index += 1  # Move to the next target frame
                self.stop_movement()

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform error: {e}")

    def stop_movement(self):
        """Stop the robot by sending zero velocities."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = ur5.base_link_name()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # Publish the stop command multiple times to ensure it registers
        for _ in range(5):
            self.twist_pub.publish(twist_msg)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.1))

def main(args=None):
    rclpy.init(args=args)
    node = UR5Navigator()
    
    # Run node in a single-threaded executor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_movement()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
