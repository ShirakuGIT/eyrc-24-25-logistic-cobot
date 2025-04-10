#!/usr/bin/env python3
"""
Script to navigate the UR5's end effector to specified target frames using MoveIt Servo.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from geometry_msgs.msg import TwistStamped, TransformStamped
from pymoveit2.robots import ur5
from math import sqrt
import tf_transformations


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
        self.target_frames = [f'obj_{i}' for i in range(4, 11)]  # Modify range as needed
        self.current_target_index = 0  # Start with the first target

        # Create a timer to call the movement function at a fixed rate
        self.timer = self.create_timer(2.0, self.navigate_to_target)  # 50 Hz

    import tf_transformations  # Ensure this is installed in your environment

    def navigate_to_target(self):
        if self.current_target_index >= len(self.target_frames):
            self.get_logger().info("All targets have been reached.")
            self.destroy_timer(self.timer)
            return

        target_frame = self.target_frames[self.current_target_index]

        try:
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
            dx, dy, dz = position[0] - eef_position[0], position[1] - eef_position[1], position[2] - eef_position[2]

            # Calculate rotational difference as angular velocity
            target_to_eef_quat = tf_transformations.quaternion_multiply(
                tf_transformations.quaternion_inverse(eef_quat), target_quat
            )
            angular_velocities = tf_transformations.euler_from_quaternion(target_to_eef_quat)

            # Create and publish the Twist message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            speed_scale = 0.5  # Scale down speeds as needed to avoid fast motions

            twist_msg.twist.linear.x = speed_scale * dx
            twist_msg.twist.linear.y = speed_scale * dy
            twist_msg.twist.linear.z = speed_scale * dz
            twist_msg.twist.angular.x = speed_scale * angular_velocities[0]
            twist_msg.twist.angular.y = speed_scale * angular_velocities[1]
            twist_msg.twist.angular.z = speed_scale * angular_velocities[2]

            self.twist_pub.publish(twist_msg)

            # Check if near target
            if sqrt(dx**2 + dy**2 + dz**2) < 0.05:
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
    navigator = UR5Navigator()

    # Use SingleThreadedExecutor to run the node
    executor = SingleThreadedExecutor()
    executor.add_node(navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop_movement()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
