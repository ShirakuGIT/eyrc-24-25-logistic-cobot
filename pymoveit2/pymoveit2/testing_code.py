import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import time
from moveit2_servo import MoveIt2Servo  # Ensure this import path is correct based on your project structure

class UR5Navigator(Node):
    def __init__(self):
        super().__init__('ur5_navigator')
        
        # Set up TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Define a list of object frames to navigate to
        self.target_frames = [f'obj_{i}' for i in range(1, 11)]  # Modify range as needed

        # Initialize the MoveIt2Servo interface
        self.servo = MoveIt2Servo(
            node=self,
            frame_id='base_link',
            linear_speed=1.0,  # Adjust speed as necessary
            angular_speed=1.0,
            enable_at_init=True,
            callback_group=self.default_callback_group
        )

        # Start MoveIt2 Servo if it’s not already started
        self.enable_servo()

        # Timer to check and navigate to transforms periodically
        self.timer = self.create_timer(2.0, self.navigate_to_transforms)

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

    def navigate_to_transforms(self):
        for target_frame in self.target_frames:
            try:
                # Lookup transform from base_link to target frame
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    'base_link', target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
                )
                
                # Extract translation and rotation from the transform
                position = (transform.transform.translation.x, 
                            transform.transform.translation.y, 
                            transform.transform.translation.z)
                orientation = (transform.transform.rotation.x, 
                               transform.transform.rotation.y, 
                               transform.transform.rotation.z)

                # Log the target frame and its position
                self.get_logger().info(f"Navigating to '{target_frame}' at position {position}")

                # Move the end effector to the target pose using servoing
                self.servo.servo(linear=position, angular=orientation)

                # Wait for a bit before moving to the next target
                time.sleep(2)

            except tf2_ros.LookupException:
                self.get_logger().warn(f"Transform from 'base_link' to '{target_frame}' not found.")
            except tf2_ros.ExtrapolationException:
                self.get_logger().warn("Extrapolation error occurred.")

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
