#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*               ===============================================
*                       Logistic coBot (LB) Theme (eYRC 2024-25)
*               ===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ LB#1425 ]
# Author List:      [ Shivaram Kumar Jagannathan ]
# Filename:         task1b_boiler_plate.py
# Functions:        color_image_callback, depth_image_callback, detect_aruco_markers, depth_image_callback
#                   process_image, estimate_aruco_pose, draw_markers, publish_transform    
#                   [ Comma separated list of functions in this file ]
# Nodes:            Add your publishing and subscribing node
#                   Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /camera/aligned_depth_to_color/image_raw ]


################### IMPORT MODULES #######################

import rclpy
import cv2
import numpy as np
import tf2_ros
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
import os
from scipy.spatial.transform import Rotation as R
import sys
from geometry_msgs.msg import PoseStamped

class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        # Initialize marker size (in meters)
        self.marker_size = 0.15  # Adjust based on your actual marker size

        # Define predefined camera matrix and distortion coefficients
        # Example values; replace with your camera's actual parameters if known
        fx = 931.1829833984375  # Focal length in pixels along X-axis
        fy = 931.1829833984375 # Focal length in pixels along Y-axis
        cx = 640.0  # Principal point X-coordinate (assuming 640x480 resolution)
        cy = 360.0  # Principal point Y-coordinate

        # Camera matrix
        self.cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

        # Distortion coefficients: [k1, k2, p1, p2, k3]
        self.dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

        # Dictionary to store previous quaternions for each marker
        self.previous_quaternions = {}

        # Log camera parameters
        self.get_logger().info("Camera Calibration Parameters:")
        self.get_logger().info(f"Camera Matrix:\n{self.cam_mat}")
        self.get_logger().info(f"Distortion Coefficients:\n{self.dist_mat}")

        # Subscribing to camera topics
        self.color_cam_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_image_callback, 10)
        self.depth_cam_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_image_callback, 10)
        
        self.get_logger().info("Subscribed to /camera/color/image_raw and /camera/aligned_depth_to_color/image_raw.")

        # OpenCV bridge to convert between ROS and OpenCV formats
        self.bridge = CvBridge()

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # TF Buffer and Listener for looking up transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # Image Data
        self.color_image = None
        self.depth_image = None
        self.pose_publisher = self.create_publisher(PoseStamped, 'aruco_marker_pose', 10)


    def publish_pose(self, marker_id, position, orientation):
        """Publish the detected pose as a PoseStamped message."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'  # Adjust based on your frame

        # Set the position and orientation directly
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]

        self.pose_publisher.publish(pose_msg)


    def depth_image_callback(self, msg):
        """Callback to process depth image from the camera"""
        try:
            # Convert the ROS image into OpenCV format
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.get_logger().debug("Received depth image.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")


    def color_image_callback(self, msg):
        """Callback to process color image from the camera"""
        try:
            # Convert the ROS image into a format that OpenCV can work with
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().debug("Received color image.")
            
            # Process the image to detect Aruco markers
            self.process_image()
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {str(e)}")

    def detect_aruco_markers(self, frame):
        """Detect aruco markers in the provided frame."""
        try:
            # Define the Aruco dictionary and parameters
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            aruco_params = cv2.aruco.DetectorParameters()
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

            # Detect markers in the frame
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)    

            if ids is not None:
                # Calculate marker centers for further processing
                centers = [np.mean(corner, axis=1) for corner in corners]
                return ids, centers, corners
            else:
                return None, None, None
        except Exception as e:
            self.get_logger().error(f"Aruco detection failed: {str(e)}")
            return None, None, None

    def depth_image_callback(self, msg):
        """Callback to process depth image from the camera"""
        try:
            # Convert the ROS image into OpenCV format
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.get_logger().debug("Received depth image.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")

    def process_image(self):
        """Process image, detect aruco markers and finally publish the transforms"""
        if self.color_image is not None and self.depth_image is not None:
            #self.get_logger().info("Processing image for Aruco detection.")

            # Detect aruco markers in the color image
            ids, centers, corners = self.detect_aruco_markers(self.color_image)

            if ids is not None:
                #self.get_logger().info(f"Aruco markers detected: {ids.flatten()}")

                # Draw detected markers on the image
                self.draw_markers(self.color_image, ids, centers, corners)

                # Publish the transform for each marker
                for i in range(len(ids)):
                    self.publish_transform(ids[i][0], centers[i], corners[i])

                # Optionally save the image for debugging
                output_path = os.path.expanduser('~/aruco_detection_output.jpg')
                cv2.imwrite(output_path, self.color_image)
                #self.get_logger().info(f"Saved detection image to {output_path}")
            else:
                self.get_logger().info("No Aruco markers detected.")
        else:
            self.get_logger().warn("Color or depth image not available to process.")


    def estimate_aruco_pose(self, corners):
        """Estimate the pose of aruco markers using calibrated camera parameters."""
        try:
            # Use the loaded camera matrix and distortion coefficients
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.cam_mat, self.dist_mat)
            return rvecs, tvecs
        except Exception as e:
            self.get_logger().error(f"Pose estimation failed: {str(e)}")
            return None, None

    def draw_markers(self, frame, ids, centers, corners):
        """Draw detected Aruco markers and their centers on the image."""
        if ids is not None:
            # Draw markers on the image
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Loop through detected markers and draw their centers
            for i, center in enumerate(centers):
                center_point = (int(center[0][0]), int(center[0][1]))
                cv2.circle(frame, center_point, 5, (0, 255, 0), -1)  # Green dot at center
                cv2.putText(frame, f'ID: {ids[i][0]}',
                            (center_point[0] + 10, center_point[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # Blue text with marker ID

            # Display the image
            cv2.imshow('Aruco Markers Detection', frame)
            cv2.waitKey(1)  # Wait for 1ms to refresh the image window
        else:
            self.get_logger().info("No Aruco markers to draw.")


    # Helper function to convert Transform to transformation matrix:       
    def transform_to_matrix(self, transform):
        translation = transform.translation
        rotation = transform.rotation

        # Convert rotation to rotation matrix
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        rot_matrix = R.from_quat(quat).as_matrix()

        # Build the transformation matrix
        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = [translation.x, translation.y, translation.z]

        return T

    def publish_transform(self, marker_id, center, corners):
        """Publish the transform for the detected Aruco marker."""
        # Wait until both color and depth images are available
        if self.color_image is None or self.depth_image is None:
            self.get_logger().warn("Color or depth image not available.")
            return

        # Get the center point coordinates
        cX = int(center[0][0])
        cY = int(center[0][1])

        # Get the depth value at the center point
        depth_value = self.depth_image[cY, cX]

        # Handle zero or invalid depth values
        if depth_value == 0 or np.isnan(depth_value):
            self.get_logger().warn(f"Invalid depth value at ({cX}, {cY}).")
            return

        # Check the encoding and convert depth accordingly
        if self.depth_image.dtype == np.uint16:
            # Depth is in millimeters
            z = depth_value / 1000.0
        elif self.depth_image.dtype == np.float32:
            # Depth is in meters
            z = depth_value
        else:
            self.get_logger().error(f"Unexpected depth image data type: {self.depth_image.dtype}")
            return

        # Print depth value for debugging
        #self.get_logger().info(f"Depth at ({cX}, {cY}): {z} meters")

        # Camera intrinsics
        centerCamX = self.cam_mat[0, 2]  # cx from camera matrix
        centerCamY = self.cam_mat[1, 2]  # cy from camera matrix
        focalX = self.cam_mat[0, 0]      # fx from camera matrix
        focalY = self.cam_mat[1, 1]      # fy from camera matrix



        # Compute x and y in OpenCV camera frame
        x = z * (cX - centerCamX) / focalX
        y = z * (cY - centerCamY) / focalY

        # Transform position from OpenCV to ROS camera frame
        x_ros = z
        y_ros = -x
        z_ros = -y

        # Extract rotation vector and translation vector from ArUco detection
        rvecs, tvecs = self.estimate_aruco_pose([corners])
        if rvecs is None or tvecs is None:
            self.get_logger().error("Pose estimation returned None.")
            return

        # Convert rotation vector to rotation matrix (OpenCV frame)
        rotation_matrix_cv, _ = cv2.Rodrigues(rvecs[0])

        # Apply the camera-to-ROS transformation matrix
        R_cv_to_ros = np.array([[0, 0, -1],
                                [1, 0, 0],
                                [0, 1, 0]])
        rotation_matrix_ros = R_cv_to_ros @ rotation_matrix_cv

        # Invert Z-axis to point into the box (optional, if necessary)
        #rotation_matrix_ros[:, 2] *= -1
        rotation_matrix_ros[:, 1] *= -1
        #rotation_matrix_ros[:, 0] *= -1


        # Build the transformation matrix from ROS camera frame to marker
        T_camera_marker = np.eye(4)
        T_camera_marker[:3, :3] = rotation_matrix_ros
        T_camera_marker[:3, 3] = [x_ros, y_ros, z_ros]

        # Now, lookup the transform from base_link to camera_link
        try:
            now = self.get_clock().now().to_msg()
            trans = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
        except tf2_ros.LookupException as ex:
            self.get_logger().error(f'Could not transform base_link to camera_link: {ex}')
            return
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f'Extrapolation exception: {ex}')
            return

        # Convert the transform to a transformation matrix (T_base_camera)
        T_base_camera = self.transform_to_matrix(trans.transform)

        # Compute the transformation from base_link to marker (T_base_marker)
        T_base_marker = T_base_camera @ T_camera_marker

        # Extract translation and rotation
        tvec_total = T_base_marker[:3, 3]
        rot_matrix_total = T_base_marker[:3, :3]
        quat_total = R.from_matrix(rot_matrix_total).as_quat()


        # Check for orientation flip by comparing with previous quaternion
        if marker_id in self.previous_quaternions:
            prev_quat = self.previous_quaternions[marker_id]
            dot_product = np.dot(quat_total, prev_quat)
            if dot_product < 0.0:
                # Correct the flip by inverting the quaternion sign
                quat_total = -quat_total

        # Update stored quaternion
        self.previous_quaternions[marker_id] = quat_total



        # Create the TransformStamped message from base_link to obj_<marker_id>
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'  # Parent frame
        transform.child_frame_id = f'obj_{marker_id}'  # Child frame

        # Set translation
        transform.transform.translation.x = float(tvec_total[0])
        transform.transform.translation.y = float(tvec_total[1])
        transform.transform.translation.z = float(tvec_total[2])

        # Set rotation (orientation)
        transform.transform.rotation.x = float(quat_total[0])
        transform.transform.rotation.y = float(quat_total[1])
        transform.transform.rotation.z = float(quat_total[2])
        transform.transform.rotation.w = float(quat_total[3])

        # Broadcast the transform from base_link to obj_<marker_id>
        self.tf_broadcaster.sendTransform(transform)



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Aruco Detection Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Close OpenCV windows gracefully
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
