#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry, Path
# import math

# class ShapeFollowingNode(Node):
#     def __init__(self):
#         super().__init__('shape_following_node')
#         self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
#         self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

#         # Publishers for paths in different frames
#         self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
#         self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
#         self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.linear_velocity = 0.2
#         self.angular_velocity = 0.5
#         self.current_angle = 0
#         self.turning_phase = False  # True: Turning, False: Moving in a rectangular path

#         # Paths for different frames
#         self.path_base_link = Path()
#         self.path_base_link.header.frame_id = 'odom'

#         self.path_base_footprint_ekf = Path()
#         self.path_base_footprint_ekf.header.frame_id = 'odom'

#         self.path_base_footprint_noisy = Path()
#         self.path_base_footprint_noisy.header.frame_id = 'odom'

#     def timer_callback(self):
#         twist_msg = Twist()

#         if self.turning_phase:
#             twist_msg.angular.z = self.angular_velocity
#             self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
#             if self.current_angle >= math.pi:  # Half circle
#                 # Switch to moving in a rectangular path phase
#                 self.turning_phase = False
#                 self.current_angle = 0
#         else:
#             # Move in a straight line for the rectangle
#             if self.current_angle < math.pi:  # First half of the rectangle
#                 twist_msg.linear.x = self.linear_velocity
#             elif math.pi <= self.current_angle < 2 * math.pi:  # Second half of the rectangle
#                 twist_msg.linear.x = 0
#             elif 2 * math.pi <= self.current_angle < 3 * math.pi:  # First half of the second circle
#                 twist_msg.angular.z = self.angular_velocity
#             elif 3 * math.pi <= self.current_angle < 4 * math.pi:  # Second half of the second circle
#                 twist_msg.angular.z = 0
#             else:  # Completed one round
#                 self.turning_phase = True
#                 self.current_angle = 0

#         # Publish cmd_vel
#         self.publisher_cmd_vel.publish(twist_msg)

#         # Publish odom
#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'
#         odom_msg.twist.twist = twist_msg
#         self.publisher_odom.publish(odom_msg)

#         # Update the paths
#         pose = PoseStamped()
#         pose.header.stamp = odom_msg.header.stamp
#         pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

#         # Update path_base_link
#         pose.pose.position.x = odom_msg.pose.pose.position.x
#         pose.pose.position.y = odom_msg.pose.pose.position.y
#         pose.pose.orientation = odom_msg.pose.pose.orientation
#         self.path_base_link.poses.append(pose)
#         self.publisher_path_base_link.publish(self.path_base_link)

#         # Update path_base_footprint_ekf
#         # Modify this part based on the transformation between base_footprint_ekf and odom
#         pose.header.frame_id = 'base_footprint_ekf'
#         self.path_base_footprint_ekf.poses.append(pose)
#         self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

#         # Update path_base_footprint_noisy
#         # Modify this part based on the transformation between base_footprint_noisy and odom
#         pose.header.frame_id = 'base_footprint_noisy'
#         self.path_base_footprint_noisy.poses.append(pose)
#         self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

# def main(args=None):
#     rclpy.init(args=args)
#     shape_following_node = ShapeFollowingNode()
#     rclpy.spin(shape_following_node)
#     shape_following_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry, Path
# import math

# class CircularMotionNode(Node):
#     def __init__(self):
#         super().__init__('circular_motion_node')
#         self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
#         self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

#         # Publishers for paths in different frames
#         self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
#         self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
#         self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.linear_velocity = 0.2
#         self.angular_velocity = -0.65
#         self.target_angle = math.radians(45)  # Target angle in radians
#         self.current_angle = 0
#         self.turning_phase = False  # True: Turning, False: Moving in a circular path

#         # Paths for different frames
#         self.path_base_link = Path()
#         self.path_base_link.header.frame_id = 'base_link'

#         self.path_base_footprint_ekf = Path()
#         self.path_base_footprint_ekf.header.frame_id = 'base_footprint_ekf'

#         self.path_base_footprint_noisy = Path()
#         self.path_base_footprint_noisy.header.frame_id = 'base_footprint_noisy'

#     def timer_callback(self):
#         twist_msg = Twist()

#         if self.turning_phase:
#             twist_msg.angular.z = self.angular_velocity
#             self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
#             if self.current_angle >= self.target_angle:
#                 # Switch to circular motion phase
#                 self.turning_phase = False
#                 self.current_angle = 0
#         else:
#             twist_msg.linear.x = self.linear_velocity
#             twist_msg.angular.z = self.angular_velocity

#         # Publish cmd_vel
#         self.publisher_cmd_vel.publish(twist_msg)

#         # Publish odom
#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'
#         odom_msg.twist.twist = twist_msg
#         self.publisher_odom.publish(odom_msg)

#         # Update the paths
#         pose = PoseStamped()
#         pose.header.stamp = odom_msg.header.stamp
#         pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

#         # Update path_base_link
#         pose.pose.position.x = odom_msg.pose.pose.position.x
#         pose.pose.position.y = odom_msg.pose.pose.position.y
#         pose.pose.orientation = odom_msg.pose.pose.orientation
#         self.path_base_link.poses.append(pose)
#         self.publisher_path_base_link.publish(self.path_base_link)

#         # Update path_base_footprint_ekf
#         # Modify this part based on the transformation between base_footprint_ekf and odom
#         pose.header.frame_id = 'base_footprint_ekf'
#         self.path_base_footprint_ekf.poses.append(pose)
#         self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

#         # Update path_base_footprint_noisy
#         # Modify this part based on the transformation between base_footprint_noisy and odom
#         pose.header.frame_id = 'base_footprint_noisy'
#         self.path_base_footprint_noisy.poses.append(pose)
#         self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

#         # Print path data for debugging
#         self.get_logger().info('Path data:')
#         self.print_path_data(self.path_base_link)

#     def print_path_data(self, path):
#         for i, pose in enumerate(path.poses):
#             self.get_logger().info(f'Pose {i}: x={pose.pose.position.x}, y={pose.pose.position.y}')

# def main(args=None):
#     rclpy.init(args=args)
#     circular_motion_node = CircularMotionNode()
#     rclpy.spin(circular_motion_node)
#     circular_motion_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry, Path
# import math
# import numpy as np

# class PathPlanningNode(Node):
#     def __init__(self):
#         super().__init__('path_planning_node')
#         self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
#         self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
#         self.publisher_planned_path = self.create_publisher(Path, 'planned_path', 10)
#         self.publisher_initial_pose = self.create_publisher(PoseStamped, 'initial_pose_estimate', 10)
#         self.subscription_goal_pose = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.linear_velocity = 0.2
#         self.angular_velocity = -0.65
#         self.target_pose = None
#         self.path = None

#         # Publish the initial pose estimate
#         initial_pose = PoseStamped()
#         initial_pose.header.frame_id = 'map'
#         initial_pose.pose.position.x = 0.0
#         initial_pose.pose.position.y = 0.0
#         initial_pose.pose.position.z = 0.0
#         initial_pose.pose.orientation.x = 0.0
#         initial_pose.pose.orientation.y = 0.0
#         initial_pose.pose.orientation.z = 0.0
#         initial_pose.pose.orientation.w = 1.0
#         self.publisher_initial_pose.publish(initial_pose)

#     def goal_pose_callback(self, msg):
#         self.target_pose = msg

#     def timer_callback(self):
#         if self.target_pose is not None:
#             # Perform path planning to the target pose
#             self.path = self.plan_path_to_pose(self.target_pose)
#             if self.path is not None:
#                 # Execute the planned path
#                 self.execute_path(self.path)

#     def plan_path_to_pose(self, pose):
#         # Perform path planning to the target pose
#         # Replace this with your path planning algorithm
#         # For example, you can use RRT or any other algorithm
#         # Return the planned path as a list of PoseStamped messages
#         planned_path = Path()
#         planned_path.header.frame_id = 'map'
#         # Add some sample poses for demonstration
#         num_poses = 10
#         for i in range(num_poses):
#             pose_stamped = PoseStamped()
#             pose_stamped.header.frame_id = 'map'
#             pose_stamped.pose.position.x = pose.pose.position.x + i * 0.1
#             pose_stamped.pose.position.y = pose.pose.position.y + i * 0.1
#             pose_stamped.pose.position.z = pose.pose.position.z
#             pose_stamped.pose.orientation.x = pose.pose.orientation.x
#             pose_stamped.pose.orientation.y = pose.pose.orientation.y
#             pose_stamped.pose.orientation.z = pose.pose.orientation.z
#             pose_stamped.pose.orientation.w = pose.pose.orientation.w
#             planned_path.poses.append(pose_stamped)
#         return planned_path

#     def execute_path(self, path):
#         # Execute the planned path
#         # For demonstration purposes, we will print the path
#         for pose_stamped in path.poses:
#             self.get_logger().info('Executing Pose: x=%f, y=%f', pose_stamped.pose.position.x, pose_stamped.pose.position.y)
#             # Perform motion control to reach each pose
#             # For example, you can use PID controller or any other control method

# def main(args=None):
#     rclpy.init(args=args)
#     path_planning_node = PathPlanningNode()
#     rclpy.spin(path_planning_node)
#     path_planning_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#LIDAR and obstacle avoidance
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import math
import random 

class RandomMotionNode(Node):
    def __init__(self):
        super().__init__('random_motion_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser_controller/out',  # Replace 'scan' with the actual topic of your lidar data
            self.lidar_callback,
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.65
        self.robot_width = 0.4  # Width of the robot
        self.obstacle_distance_threshold = 0.6  # Adjust based on the range of your lidar
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'base_link'
        self.obstacle_detected = True

    def timer_callback(self):
        twist_msg = Twist()
        if not self.obstacle_detected:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = random.uniform(-self.angular_velocity, self.angular_velocity)
        else:
            # Calculate the required rotation angle to cover the new path width
            rotation_angle = math.atan2(self.robot_width / 2, self.obstacle_distance_threshold)
            twist_msg.angular.z = math.copysign(self.angular_velocity, rotation_angle)
            self.obstacle_detected = False
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

        # Update path_base_link
        pose = PoseStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.orientation = odom_msg.pose.pose.orientation
        self.path_base_link.poses.append(pose)
        self.publisher_path_base_link.publish(self.path_base_link)

    def lidar_callback(self, msg):
        # Check for obstacles within a threshold distance
        for i, range in enumerate(msg.ranges):
            if range < self.obstacle_distance_threshold:
                self.obstacle_detected = True
                break

def main(args=None):
    rclpy.init(args=args)
    random_motion_node = RandomMotionNode()
    rclpy.spin(random_motion_node)
    random_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







# # CIRCULAR MOTION ONLY
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry, Path
# import math

# class CircularMotionNode(Node):
#     def __init__(self):
#         super().__init__('circular_motion_node')
#         self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
#         self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

#         # Publishers for paths in different frames
#         self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
#         self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
#         self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.linear_velocity = 0.2
#         self.angular_velocity = 0.65
#         self.target_angle = math.radians(45)  # Target angle in radians
#         self.current_angle = 0
#         self.turning_phase = False  # True: Turning, False: Moving in a circular path

#         # Paths for different frames
#         self.path_base_link = Path()
#         self.path_base_link.header.frame_id = 'base_link'

#         self.path_base_footprint_ekf = Path()
#         self.path_base_footprint_ekf.header.frame_id = 'base_footprint_ekf'

#         self.path_base_footprint_noisy = Path()
#         self.path_base_footprint_noisy.header.frame_id = 'base_footprint_noisy'

#     def timer_callback(self):
#         twist_msg = Twist()

#         if self.turning_phase:
#             twist_msg.angular.z = self.angular_velocity
#             self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
#             if self.current_angle >= self.target_angle:
#                 # Switch to circular motion phase
#                 self.turning_phase = False
#                 self.current_angle = 0
#         else:
#             twist_msg.linear.x = self.linear_velocity
#             twist_msg.angular.z = self.angular_velocity

#         # Publish cmd_vel
#         self.publisher_cmd_vel.publish(twist_msg)

#         # Publish odom
#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'
#         odom_msg.twist.twist = twist_msg
#         self.publisher_odom.publish(odom_msg)

#         # Update the paths
#         pose = PoseStamped()
#         pose.header.stamp = odom_msg.header.stamp
#         pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

#         # Update path_base_link
#         pose.pose.position.x = odom_msg.pose.pose.position.x
#         pose.pose.position.y = odom_msg.pose.pose.position.y
#         pose.pose.orientation = odom_msg.pose.pose.orientation
#         self.path_base_link.poses.append(pose)
#         self.publisher_path_base_link.publish(self.path_base_link)

#         # Update path_base_footprint_ekf
#         # Modify this part based on the transformation between base_footprint_ekf and odom
#         pose.header.frame_id = 'base_footprint_ekf'
#         self.path_base_footprint_ekf.poses.append(pose)
#         self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

#         # Update path_base_footprint_noisy
#         # Modify this part based on the transformation between base_footprint_noisy and odom
#         pose.header.frame_id = 'base_footprint_noisy'
#         self.path_base_footprint_noisy.poses.append(pose)
#         self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

# def main(args=None):
#     rclpy.init(args=args)
#     circular_motion_node = CircularMotionNode()
#     rclpy.spin(circular_motion_node)
#     circular_motion_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
