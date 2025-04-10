from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time
from geometry_msgs.msg import PoseStamped
import math
from scipy.spatial.transform import Rotation as R  # Use scipy instead of tf_transformations

rclpy.init()

# Initialize the navigator
nav = BasicNavigator()

# Functions for quaternion and Euler conversions using scipy
def quaternion_from_euler(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # Returns [x, y, z, w]
    return q

def euler_from_quaternion(orientation_q):
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(quaternion)
    roll, pitch, yaw = r.as_euler('xyz')
    return yaw  # Return yaw angle

# Define the poses
def create_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    # Convert yaw to quaternion
    q = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

# Function to extract yaw from a pose
def get_yaw_from_pose(pose):
    orientation = pose.pose.orientation
    yaw = euler_from_quaternion(orientation)
    return yaw

# Define initial pose and waypoints
initial_pose = create_pose(1.84, -9.05, 3.14)
P1 = create_pose(-0.12, -2.35, 3.14)
P2 = create_pose(1.86, 2.56, 0.97)
P3 = create_pose(-3.84, 2.64, 2.78)

# Set initial pose and activate the navigation
nav.setInitialPose(initial_pose)
nav.waitUntilNav2Active()

# List of poses to navigate through
poses = [P1, P2, P3]

# Tolerances
position_tolerance = 0.3  # meters
orientation_tolerance = math.radians(10)  # radians (10 degrees)

# Function to navigate to a pose, log actual pose, and calculate errors
def navigate_to_pose(pose, pose_number):
    print(f"\nNavigating to Pose {pose_number}: x={pose.pose.position.x}, y={pose.pose.position.y}, yaw={math.degrees(get_yaw_from_pose(pose))} degrees")
    nav.goToPose(pose)

    # Variables to store the last feedback
    last_feedback = None

    # Wait for the task to complete
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            # Store the latest feedback
            last_feedback = feedback
            #print("Feedback:", feedback)

    # Check the result of the navigation
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"Successfully reached Pose {pose_number}!")
    elif result == TaskResult.CANCELED:
        print(f"Navigation to Pose {pose_number} was canceled!")
        return
    else:
        print(f"Failed to reach Pose {pose_number}.")
        return

    # Use the last feedback to get the robot's current pose
    if last_feedback and hasattr(last_feedback, 'current_pose'):
        current_pose = last_feedback.current_pose
    else:
        print("No feedback received. Cannot get current pose.")
        return

    # Log the actual pose
    actual_x = current_pose.pose.position.x
    actual_y = current_pose.pose.position.y
    actual_yaw = get_yaw_from_pose(current_pose)
    desired_x = pose.pose.position.x
    desired_y = pose.pose.position.y
    desired_yaw = get_yaw_from_pose(pose)

    print(f"Actual Pose at Pose {pose_number}: x={actual_x:.2f}, y={actual_y:.2f}, yaw={math.degrees(actual_yaw):.2f} degrees")

    # Calculate position error
    delta_x = actual_x - desired_x
    delta_y = actual_y - desired_y
    position_error = math.hypot(delta_x, delta_y)

    # Calculate orientation error
    orientation_error = actual_yaw - desired_yaw
    orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))  # Normalize between -pi and pi
    orientation_error_deg = math.degrees(abs(orientation_error))

    # Check if within tolerances
    position_within_tolerance = position_error <= position_tolerance
    orientation_within_tolerance = abs(orientation_error) <= orientation_tolerance

    print(f"Position Error: {position_error:.3f} meters")
    print(f"Orientation Error: {orientation_error_deg:.2f} degrees")

    if position_within_tolerance:
        print(f"Position is within tolerance of ±{position_tolerance} meters.")
    else:
        print(f"Position is NOT within tolerance of ±{position_tolerance} meters.")

    if orientation_within_tolerance:
        print(f"Orientation is within tolerance of ±{math.degrees(orientation_tolerance)} degrees.")
    else:
        print(f"Orientation is NOT within tolerance of ±{math.degrees(orientation_tolerance)} degrees.")

    # Wait for 2 seconds before moving to the next pose
    print("Waiting for 2 seconds before moving to the next pose...")
    time.sleep(2)

# Go through each pose one by one
for idx, pose in enumerate(poses):
    navigate_to_pose(pose, idx + 1)

print("Completed navigation through all poses.")

# Shutdown the navigator
nav.lifecycleShutdown()
rclpy.shutdown()

