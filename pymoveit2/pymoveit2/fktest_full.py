#!/usr/bin/env python3
"""
Script to move the UR5 robot through a sequence of poses to pick up and drop multiple boxes,
including gripper magnet service calls, using forward kinematics and object scanning.
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
import tf_transformations
import math
import time


class UR5JointServoController(Node):
    def __init__(self):
        super().__init__('ur5_joint_servo_controller')

        # Possible states
        self.state = 'lookup_objects'  # Start with looking up objects

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
        self.drop_angles_deg = [-14, -72, 136, -157, -89, 180]

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
        self.joint_velocity_gain = 200.0  # Adjust as needed
        self.max_joint_velocity = 200.0  # radians per second

        # Timer to send joint commands
        self.timer = self.create_timer(0.02, self.send_joint_command)  # 50 Hz

        # Create service clients for gripper magnet
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        # Initialize object handling
        self.box_ids = []  # List of obj_<id>s to process
        self.processed_boxes = []  # List of obj_<id>s already processed
        self.current_box_index = 0
        self.box_name = ''

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # UR5 robot DH parameters (for forward kinematics)
        self.dh_params = {
            'a': [0, -0.42500, -0.39225, 0, 0, 0],
            'alpha': [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0],
            'd': [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
        }

        # Movement iteration control
        self.iteration = 0
        self.max_iterations = 2  # Adjust as needed

        self.get_logger().info('UR5 Joint Servo Controller Initialized.')

    def joint_state_callback(self, msg):
        # Update current joint positions
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def send_joint_command(self):
        if len(self.current_joint_positions) < 6:
            return

        # State machine for controlling the robot
        if self.state == 'lookup_objects':
            self.lookup_objects()
            if self.box_ids:
                self.get_logger().info(f'Found objects to process: {self.box_ids}')
                self.current_box_index = 0
                self.state = 'decide_pickup'
            else:
                self.get_logger().info('No new objects found. Waiting for new objects.')
                # Optionally, wait for some time before checking again
                time.sleep(1.0)
                # You can decide to set state to 'done' if no new objects are expected
                # self.state = 'done'

        elif self.state == 'decide_pickup':
            if self.current_box_index >= len(self.box_ids):
                # All current objects have been processed, look for new ones
                self.state = 'lookup_objects'
                return

            current_id = self.box_ids[self.current_box_index]
            self.box_name = f'box{current_id}'
            obj_frame = f'obj_{current_id}'

            # Get the object's position
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    obj_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                # Use forward kinematics to get positions of pickup1 and pickup2
                pickup1_fk = self.forward_kinematics(self.pickup1_angles_rad)
                pickup2_fk = self.forward_kinematics(self.pickup2_angles_rad)

                pickup1_pos = pickup1_fk['position']
                pickup2_pos = pickup2_fk['position']

                # Compute distances
                obj_pos = np.array([x, y, z])
                dist_to_pickup1 = np.linalg.norm(obj_pos - pickup1_pos)
                dist_to_pickup2 = np.linalg.norm(obj_pos - pickup2_pos)

                # Decide pickup sequence based on minimum distance
                if dist_to_pickup1 > dist_to_pickup2:
                    self.pickup_sequence = 'pickup1'
                    self.get_logger().info(f'Object {obj_frame} is closer to pickup1 area.')
                    self.state = 'move_to_pickup1'
                else:
                    self.pickup_sequence = 'pickup2'
                    self.get_logger().info(f'Object {obj_frame} is closer to pickup2 area.')
                    self.state = 'move_to_pickup2'

            except Exception as e:
                self.get_logger().error(f'Error getting transform for {obj_frame}: {e}')
                # Move to next object
                self.current_box_index += 1

        elif self.state == 'move_to_pickup1':
            self.target_joint_angles = self.pickup1_angles_rad
            self.state = 'execute_pickup1'

        elif self.state == 'execute_pickup1':
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached pickup1 pose.')
                self.get_logger().info(f'Attaching {self.box_name}.')
                self.gripper_attach()
                self.state = 'move_to_pickup1_intermediate'

        elif self.state == 'move_to_pickup1_intermediate':
            self.target_joint_angles = self.pickup1_intermediate_angles_rad
            self.state = 'execute_pickup1_intermediate'

        elif self.state == 'execute_pickup1_intermediate':
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached pickup1 intermediate pose.')
                self.state = 'move_to_drop'

        elif self.state == 'move_to_pickup2':
            self.target_joint_angles = self.pickup2_angles_rad
            self.state = 'execute_pickup2'

        elif self.state == 'execute_pickup2':
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached pickup2 pose.')
                self.get_logger().info(f'Attaching {self.box_name}.')
                self.gripper_attach()
                self.state = 'move_to_pickup2_intermediate'

        elif self.state == 'move_to_pickup2_intermediate':
            self.target_joint_angles = self.pickup2_intermediate_angles_rad
            self.state = 'execute_pickup2_intermediate'

        elif self.state == 'execute_pickup2_intermediate':
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached pickup2 intermediate pose.')
                self.state = 'move_to_drop'

        elif self.state == 'move_to_drop':
            self.target_joint_angles = self.drop_angles_rad
            self.state = 'execute_drop'

        elif self.state == 'execute_drop':
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached drop pose.')
                self.get_logger().info(f'Detaching {self.box_name}.')
                self.gripper_detach()
                self.state = 'move_to_base'

        elif self.state == 'move_to_base':
            self.target_joint_angles = self.base_angles_rad
            self.state = 'execute_base'

        elif self.state == 'execute_base':
            self.move_to_target_joint_angles()
            if self.at_target_joint_angles():
                self.get_logger().info('Reached base pose.')
                # Mark current object as processed
                current_id = self.box_ids[self.current_box_index]
                self.processed_boxes.append(current_id)
                self.get_logger().info(f'Object {current_id} has been processed.')
                # Remove the current object from the list
                self.box_ids.pop(self.current_box_index)
                # Reset index to start from the next object
                self.current_box_index = 0
                # Move to next object
                self.state = 'decide_pickup'

        elif self.state == 'done':
            self.get_logger().info('Movement complete. Sending zero velocities to stop the robot.')
            self.stop_movement()
            self.timer.cancel()
            return

    def lookup_objects(self):
        # Perform multiple tries for scanning the ids
        found_ids = []
        for _ in range(3):  # Try 3 times
            for i in range(1, 7):  # Assuming IDs from 1 to 6
                if i in self.processed_boxes:
                    continue  # Skip already processed objects
                obj_frame = f'obj_{i}'
                try:
                    # Try to lookup the transform to see if the object exists
                    self.tf_buffer.lookup_transform(
                        'base_link',
                        obj_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    if i not in self.box_ids and i not in found_ids:
                        found_ids.append(i)
                except:
                    continue
            if found_ids:
                break  # Exit early if objects are found

        if found_ids:
            self.box_ids.extend(found_ids)

    def forward_kinematics(self, joint_angles):
        # Compute the end-effector pose given joint angles using DH parameters
        T = np.identity(4)
        for i in range(6):
            a = self.dh_params['a'][i]
            alpha = self.dh_params['alpha'][i]
            d = self.dh_params['d'][i]
            theta = joint_angles[i]

            T_i = tf_transformations.concatenate_matrices(
                tf_transformations.translation_matrix([0, 0, d]),
                tf_transformations.rotation_matrix(theta, [0, 0, 1]),
                tf_transformations.translation_matrix([a, 0, 0]),
                tf_transformations.rotation_matrix(alpha, [1, 0, 0])
            )
            T = np.dot(T, T_i)

        position = T[:3, 3]
        orientation = tf_transformations.euler_from_matrix(T)

        return {'position': position, 'orientation': orientation}

    def move_to_target_joint_angles(self):
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
        if not self.context.ok():
            return
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
