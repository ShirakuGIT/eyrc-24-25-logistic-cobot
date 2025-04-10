import rclpy
from pymoveit2 import MoveIt2
from threading import Thread
from rclpy.node import Node
import time
import tf2_ros
from geometry_msgs.msg import TransformStamped
from linkattacher_msgs.srv import AttachLink, DetachLink

class UR5Controller:
    def __init__(self, node):
        self.node = node
        self.servo = MoveIt2Servo(node=node, frame_id='base_link')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
        )

    def get_tf(self, target_frame):
        try:
            return self.tf_buffer.lookup_transform('base_link', target_frame, rclpy.time.Time())
        except:
            self.node.get_logger().info(f"Could not find transform for {target_frame}")
            return None

    def move_to_pose(self, target_tf):
        linear = (target_tf.transform.translation.x, target_tf.transform.translation.y, target_tf.transform.translation.z)
        angular = (target_tf.transform.rotation.x, target_tf.transform.rotation.y, target_tf.transform.rotation.z)
        self.servo.servo(linear=linear, angular=angular)
        
    def gripper_action(self, action, box_name):
        service = AttachLink if action == "attach" else DetachLink
        gripper_control = self.node.create_client(service, f'/GripperMagnet{action.capitalize()}')
        req = service.Request()
        req.model1_name, req.link1_name = box_name, 'link'
        req.model2_name, req.link2_name = 'ur5', 'wrist_3_link'

        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{action.capitalize()} service not available, waiting...')

        future = gripper_control.call_async(req)
        time.sleep(0.1)
        if future.done() and hasattr(future.result(), 'success') and future.result().success:
            self.node.get_logger().info(f"{action.capitalize()} succeeded for {box_name}")
        else:
            self.node.get_logger().info(f"{action.capitalize()} failed for {box_name}")

    def pick_and_place(self):
        for pick_obj, box_name in [('obj1', 'box3'), ('obj6', 'box49')]:
            pick_tf = self.get_tf(pick_obj)
            drop_tf = self.get_tf('obj12')
            if pick_tf and drop_tf:
                self.move_to_pose(pick_tf)
                self.gripper_action("attach", box_name)
                time.sleep(0.5)
                self.move_to_pose(drop_tf)
                self.gripper_action("detach", box_name)
                time.sleep(0.5)

def main():
    rclpy.init()
    node = Node("ur5_controller")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    controller = UR5Controller(node)

    try:
        controller.pick_and_place()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
