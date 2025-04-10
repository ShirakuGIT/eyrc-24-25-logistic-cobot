#!/usr/bin/env python3
"""
Script to move the UR5 robot through a sequence of poses to pick up and drop multiple boxes,
including gripper magnet service calls.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from rclpy.qos import QoSProfile
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from linkattacher_msgs.srv import AttachLink, DetachLink
import time


class UR5JointServoController(Node):
    def __init__(self):
        super().__init__('ur5_joint_servo_controller')

        # Possible states
        self.state = 'move_to_pickup1'  # Start with moving to pickup1

        # Publisher to send joint velocity commands
        self.joint_cmd_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            QoSProfile(depth=10)
        )

        # Subscriber to get current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        # Initialize current joint positions
        self.current_joint_positions = {}
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Define target joint angles in degrees
        self.base_angles_deg = [0, -137, 138, -181, -90, 180]
        self.pickup1_angles_deg = [-103, -72, 115, -134, -90, 180]
        self.pickup1_intermediate_angles_deg = [-67, -111, 102, -99, -92, 187]
        self.pickup2_angles_deg = [85, -69, 113, -139, -90, 180]
        self.pickup2_intermediate_angles_deg = [55, -88, 76, -83, -90, 180]
        self.drop_angles_deg = [-13, -44, 112, -160, -91, 180]

        # Convert target joint angles to radians
        self.base_angles_rad = np.radians(self.base_angles_deg)
        self.pickup1_angles_rad = np.radians(self.pickup1_angles_deg)
        self.pickup1_intermediate_angles_rad = np.radians(self.pickup1_intermediate_angles_deg)
        self.pickup2_angles_rad = np.radians(self.pickup2_angles_deg)
        self.pickup2_intermediate_angles_rad = np.radians(self.pickup2_intermediate_angles_deg)
        self.drop_angles_rad = np.radians(self.drop_angles_deg)

        # Set initial target as the first pickup pose
        self.target_joint_angles = self.pickup1_angles_rad

        # Control parameters
        self.joint_velocity_gain = 50.0  # Adjust as needed
        self.max_joint_velocity = 50.0  # radians per second

        # Timer to send joint commands
        self.timer = self.create_timer(0.02, self.send_joint_command)  # 50 Hz

        # Create service clients for gripper magnet
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        # Initialize box handling
        self.iteration = 0  # To keep track of the number of times the sequence has been executed
        self.max_iterations = 4  # Total number of pickups and drops
        self.box_names = ['box4', 'box1']  # List of boxes to pick up
        self.current_box_index = 0
        self.box_name = self.box_names[self.current_box_index]

        self.get_logger().info('UR5 Joint Servo Controller Initialized.')

    def joint_state_callback(self, msg):
        # Update current joint positions
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def send_joint_command(self):
        if len(self.current_joint_positions) < 6:
            return

        # Determine the target joint angles based on the current state
        if self.state == 'move_to_pickup1':
            self.target_joint_angles = self.pickup1_angles_rad
        elif self.state == 'move_to_pickup1_intermediate':
            self.target_joint_angles = self.pickup1_intermediate_angles_rad
        elif self.state == 'move_to_pickup2':
            self.target_joint_angles = self.pickup2_angles_rad
        elif self.state == 'move_to_pickup2_intermediate':
            self.target_joint_angles = self.pickup2_intermediate_angles_rad
        elif self.state == 'move_to_drop':
            self.target_joint_angles = self.drop_angles_rad
        elif self.state == 'move_to_base':
            self.target_joint_angles = self.base_angles_rad
        elif self.state == 'done':
            self.get_logger().info('Movement complete. Sending zero velocities to stop the robot.')
            # Send zero velocities
            cmd = JointJog()
            cmd.joint_names = self.joint_names
            cmd.velocities = [0.0] * len(self.joint_names)
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.joint_cmd_pub.publish(cmd)
            # Optionally, cancel the timer
            self.timer.cancel()
            return

        # Compute joint velocities
        joint_velocities = {}
        error_tolerance = 0.01  # radians
        at_target = True

        for idx, joint_name in enumerate(self.joint_names):
            error = self.target_joint_angles[idx] - self.current_joint_positions.get(joint_name, 0.0)
            if abs(error) > error_tolerance:
                at_target = False
            velocity = self.joint_velocity_gain * error
            # Limit velocity
            velocity = max(min(velocity, self.max_joint_velocity), -self.max_joint_velocity)
            joint_velocities[joint_name] = velocity

        # Check if the robot has reached the target pose
        if at_target:
            if self.state == 'move_to_pickup1':
                self.get_logger().info('Reached pickup1 pose.')
                time.sleep(0.1)
                self.get_logger().info(f'Attaching {self.box_name}.')
                self.gripper_attach()
                self.get_logger().info('Moving to drop pose.')
                self.state = 'move_to_pickup1_intermediate'
            elif self.state == 'move_to_pickup1_intermediate':
                self.get_logger().info('Reached pickup1 intermediate pose.')
                self.state = 'move_to_drop'
            elif self.state == 'move_to_pickup2':
                self.get_logger().info('Reached pickup2 pose.')
                time.sleep(0.1)
                self.get_logger().info(f'Attaching {self.box_name}.')
                self.gripper_attach()
                self.get_logger().info('Moving to drop pose.')
                self.state = 'move_to_pickup2_intermediate'
            elif self.state == 'move_to_pickup2_intermediate':
                self.get_logger().info('Reached pickup2 intermediate pose.')
                self.state = 'move_to_drop'
            elif self.state == 'move_to_drop':
                self.get_logger().info('Reached drop pose.')
                time.sleep(0.1)
                self.get_logger().info(f'Detaching {self.box_name}.')
                self.gripper_detach()
                self.get_logger().info('Moving to base pose.')
                self.state = 'move_to_base'
            elif self.state == 'move_to_base':
                self.get_logger().info('Reached base pose.')
                self.iteration += 1
                if self.iteration >= self.max_iterations:
                    self.state = 'done'
                else:
                    # Alternate between boxes and pickup positions
                    self.current_box_index = (self.current_box_index + 1) % len(self.box_names)
                    self.box_name = self.box_names[self.current_box_index]
                    if self.current_box_index == 0:
                        self.state = 'move_to_pickup1'
                    else:
                        self.state = 'move_to_pickup2'
                    self.get_logger().info(f'Switching to {self.box_name}.')
                time.sleep(0.1)  # Optional delay before next movement
            return  # Skip publishing velocities when at target

        # Publish joint velocities
        cmd = JointJog()
        cmd.joint_names = list(joint_velocities.keys())
        cmd.velocities = list(joint_velocities.values())
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.joint_cmd_pub.publish(cmd)

    def gripper_attach(self):
        req = AttachLink.Request()
        req.model1_name = self.box_name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'

        self.get_logger().info(f'Waiting for GripperMagnetON service...')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GripperMagnetON service not available, waiting...')

        self.get_logger().info(f'Calling GripperMagnetON service to attach {self.box_name}')
        future = self.attach_client.call_async(req)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        time.sleep(0.1)
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'GripperMagnetON succeeded for {self.box_name}')
                else:
                    self.get_logger().info(f'GripperMagnetON failed for {self.box_name}')
            except Exception as e:
                self.get_logger().error(f'GripperMagnetON call failed: {e}')
        else:
            self.get_logger().error(f'GripperMagnetON call did not complete within timeout')

    def gripper_detach(self):
        req = DetachLink.Request()
        req.model1_name = self.box_name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'

        self.get_logger().info(f'Waiting for GripperMagnetOFF service...')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GripperMagnetOFF service not available, waiting...')

        self.get_logger().info(f'Calling GripperMagnetOFF service to detach {self.box_name}')
        future = self.detach_client.call_async(req)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        time.sleep(0.1)
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'GripperMagnetOFF succeeded for {self.box_name}')
                else:
                    self.get_logger().info(f'GripperMagnetOFF failed for {self.box_name}')
            except Exception as e:
                self.get_logger().error(f'GripperMagnetOFF call failed: {e}')
        else:
            self.get_logger().error(f'GripperMagnetOFF call did not complete within timeout')

    def stop_movement(self):
        # Publish zero velocities to stop the robot
        joint_jog_msg = JointJog()
        joint_jog_msg.header.stamp = self.get_clock().now().to_msg()
        joint_jog_msg.header.frame_id = 'base_link'
        joint_jog_msg.joint_names = self.joint_names
        joint_jog_msg.velocities = [0.0] * len(self.joint_names)

        # Publish multiple times to ensure the robot stops
        for _ in range(10):
            self.joint_cmd_pub.publish(joint_jog_msg)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = UR5JointServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_movement()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
