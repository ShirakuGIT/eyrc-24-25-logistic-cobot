#!/usr/bin/env python3
"""
Script to move the UR5 robot to pick up and drop multiple objects using inverse kinematics and TF lookups,
using servoing commands and gripper magnet service calls.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from rclpy.qos import QoSProfile
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from linkattacher_msgs.srv import AttachLink, DetachLink
import tf2_ros
import math
import time

class UR5JointServoController(Node):
    def __init__(self):
        super().__init__('ur5_joint_servo_controller')

        # Possible states
        self.state = 'move_to_pickup'  # Start with moving to pickup

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

        # Initialize target joint angles
        self.target_joint_angles = [0.0] * 6

        # Control parameters
        self.joint_velocity_gain = 1.0  # Adjust as needed
        self.max_joint_velocity = 1.0  # radians per second

        # Timer to send joint commands
        self.timer = self.create_timer(0.02, self.send_joint_command)  # 50 Hz

        # Create service clients for gripper magnet
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        # Initialize object handling
        self.id_list = list(range(1, 7))  # IDs from 1 to 6
        self.current_id_index = 0  # Index into id_list
        self.box_name = ''
        self.obj_frame = ''

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('UR5 Joint Servo Controller Initialized.')

    def joint_state_callback(self, msg):
        # Update current joint positions
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def send_joint_command(self):
        if len(self.current_joint_positions) < 6:
            return

        # Determine the target joint angles based on the current state
        if self.state == 'move_to_pickup':
            if self.current_id_index >= len(self.id_list):
                # No more IDs to process
                self.get_logger().info('No more objects to pick up.')
                self.state = 'done'
                return

            current_id = self.id_list[self.current_id_index]
            self.obj_frame = f'obj_{current_id}'

            # Perform TF lookup to get the object position
            try:
                source_frame = 'base_link'  # Adjust if your robot's base frame is named differently
                transform = self.tf_buffer.lookup_transform(
                    source_frame,
                    self.obj_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                # Get the position of the object
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z + 0.1  # Approach 10 cm above the object

                # Compute inverse kinematics to get joint angles
                joint_angles = self.compute_ik(x, y, z)
                if joint_angles is not None:
                    self.target_joint_angles = joint_angles
                    self.box_name = f'box_{current_id}'  # For gripper magnet services
                    self.get_logger().info(f'Computed IK for {self.obj_frame}, moving to pickup.')
                    self.state = 'move_to_pickup_execute'
                else:
                    self.get_logger().error(f'Failed to compute IK for {self.obj_frame}')
                    # Remove current ID from list and try next
                    self.id_list.pop(self.current_id_index)
                    self.get_logger().info(f'Removing ID {current_id} from list due to IK failure.')
            except Exception as e:
                self.get_logger().error(f'Error in TF lookup for {self.obj_frame}: {e}')
                # Remove current ID from list and try next
                self.id_list.pop(self.current_id_index)
                self.get_logger().info(f'Removing ID {current_id} from list due to TF lookup failure.')

        elif self.state == 'move_to_pickup_execute':
            # Move to the target joint angles
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached pickup pose.')
                self.get_logger().info(f'Attaching {self.box_name}.')
                self.gripper_attach()
                self.get_logger().info('Moving to drop pose.')
                self.state = 'move_to_drop'
            return

        elif self.state == 'move_to_drop':
            # Set predefined drop joint angles
            self.target_joint_angles = np.radians([-14, -72, 136, -157, -89, 180])
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached drop pose.')
                self.get_logger().info(f'Detaching {self.box_name}.')
                self.gripper_detach()
                self.get_logger().info('Moving to base pose.')
                self.state = 'move_to_base'
            return

        elif self.state == 'move_to_base':
            # Set predefined base joint angles
            self.target_joint_angles = np.radians([0, -137, 138, -181, -90, 180])
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached base pose.')
                # Proceed to next ID
                self.current_id_index += 1
                if self.current_id_index >= len(self.id_list):
                    self.get_logger().info('No more objects to pick up.')
                    self.state = 'done'
                else:
                    self.state = 'move_to_pickup'
            return

        elif self.state == 'done':
            self.get_logger().info('Movement complete. Sending zero velocities to stop the robot.')
            self.stop_movement()
            self.timer.cancel()
            return

    def move_to_target_joint_angles(self):
        # Compute joint velocities
        joint_velocities = {}
        error_tolerance = 0.01  # radians

        for idx, joint_name in enumerate(self.joint_names):
            error = self.target_joint_angles[idx] - self.current_joint_positions.get(joint_name, 0.0)
            velocity = self.joint_velocity_gain * error
            # Limit velocity
            velocity = max(min(velocity, self.max_joint_velocity), -self.max_joint_velocity)
            joint_velocities[joint_name] = velocity

        # Publish joint velocities
        cmd = JointJog()
        cmd.joint_names = list(joint_velocities.keys())
        cmd.velocities = list(joint_velocities.values())
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.joint_cmd_pub.publish(cmd)

    def at_target_joint_angles(self):
        error_tolerance = 0.01  # radians
        at_target = True
        for idx, joint_name in enumerate(self.joint_names):
            error = abs(self.target_joint_angles[idx] - self.current_joint_positions.get(joint_name, 0.0))
            if error > error_tolerance:
                at_target = False
                break
        return at_target

    def compute_ik(self, x, y, z):
        # UR5 robot parameters
        d1 = 0.089159
        a2 = -0.42500
        a3 = -0.39225
        d4 = 0.10915
        d5 = 0.09465
        d6 = 0.0823

        # Compute wrist center position
        wx = x
        wy = y
        wz = z - d6  # Adjust for end-effector length

        # Compute inverse kinematics
        try:
            # Joint 1
            theta1 = math.atan2(wy, wx)

            # Compute parameters for theta2 and theta3
            r = math.sqrt(wx**2 + wy**2)
            s = wz - d1
            D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)

            if abs(D) > 1:
                self.get_logger().error('No valid solutions for theta3')
                return None

            # Joint 3
            theta3 = math.atan2(-math.sqrt(1 - D**2), D)  # Use the negative solution for elbow down

            # Joint 2
            theta2 = math.atan2(s, r) - math.atan2(a3 * math.sin(theta3), a2 + a3 * math.cos(theta3))

            # For simplicity, set wrist joints to zero
            theta4 = 0.0
            theta5 = 0.0
            theta6 = 0.0

            joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
            return joint_angles

        except Exception as e:
            self.get_logger().error(f'Error computing inverse kinematics: {e}')
            return None

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
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
