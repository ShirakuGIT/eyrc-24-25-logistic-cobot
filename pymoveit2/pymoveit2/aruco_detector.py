#!/usr/bin/env python3
"""
Script to detect ArUco markers and determine corresponding boxes and their pickup positions.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener
from std_msgs.msg import String
import json

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Possible marker IDs (from 1 to 10)
        self.marker_ids = range(1, 11)

        # Frame of reference
        self.base_frame = 'base_link'  # Adjust to your robot's base frame

        # Publisher to publish detected boxes
        self.publisher = self.create_publisher(String, 'detected_boxes', 10)

        # Timer to periodically check for markers
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.lookup_markers)

        self.get_logger().info('Aruco Detector Initialized.')

    def lookup_markers(self):
        detected_boxes = []
        for marker_id in self.marker_ids:
            marker_frame = f'obj_{marker_id}'
            try:
                # Lookup transform from base_frame to marker_frame
                trans = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    marker_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                # Get translation in x-direction
                x = trans.transform.translation.x

                # Determine pickup position
                if x > 0:
                    pickup_position = 'pickup1'
                else:
                    pickup_position = 'pickup2'

                # Map marker ID to box name
                box_name = f'box{marker_id}'

                # Store the information
                detected_boxes.append({
                    'box_name': box_name,
                    'pickup_position': pickup_position
                })

                # Output the information
                self.get_logger().info(f'Detected {box_name} near {pickup_position}')

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                # Marker not detected or transform not available
                pass

        # Publish the detected_boxes list to the topic
        msg = String()
        msg.data = json.dumps(detected_boxes)
        self.publisher.publish(msg)

        if detected_boxes:
            self.get_logger().info(f'Total detected boxes: {len(detected_boxes)}')
        else:
            self.get_logger().info('No markers detected.')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
