import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
import time

class SimpleUR5Move(Node):
    def __init__(self):
        super().__init__('simple_ur5_move')
        
        # Create publisher to send TwistStamped messages for movement
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # Call the service to enable the servo
        self.enable_servo()
        
        # Publish a movement command
        self.move_up()

        # Stop the movement after a short delay
        time.sleep(3)  # Move for 3 seconds
        self.stop_movement()
        
    def enable_servo(self):
        """Enable the servo by calling the start_servo service."""
        start_service = self.create_client(Trigger, '/servo_node/start_servo')
        if not start_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Servo service not available.")
            return

        # Call the start_servo service
        req = Trigger.Request()
        future = start_service.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info("Servo successfully started.")
        else:
            self.get_logger().error("Failed to start servo.")

    def move_up(self):
        """Send a command to move the robot up along the z-axis."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'  # Adjust based on your robot’s frame

        # Set linear movement along the z-axis
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.2  # Move up along z-axis
        
        # No angular movement
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        twist_msg.twist.angular.w = 0.0

        # Publish the twist message
        self.twist_publisher.publish(twist_msg)
        self.get_logger().info("Moving up...")

    def stop_movement(self):
        """Stop the robot by sending a zero twist command."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        # Set all velocities to zero
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # Publish the stop command
        self.twist_publisher.publish(twist_msg)
        self.get_logger().info("Stopping movement.")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleUR5Move()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
