#!/usr/bin/env python3

import rclpy
import yaml
import pandas as pd
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
from nav_msgs.msg import Odometry  # Import Odometry message type
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import PointStamped
import numpy as np
from rclpy.time import Time
from tf_transformations import euler_from_quaternion

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.get_logger().info('Subscriber node initialized')

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.timer = self.create_timer(2, self.timer_callback)

        self.ground_truth_data = []  # Store ground truth data
        self.estimated_data = []  # Store estimated data

        # Publishers for markers
        self.publisher_gt_marker = self.create_publisher(MarkerArray, 'ground_truth_trajectory', 10)
        self.publisher_est_marker = self.create_publisher(MarkerArray, 'estimated_trajectory', 10)

        # Subscribers for Odometry messages
        self.get_logger().info('Subscribing to ground truth Odometry topic: /diff_cont/odom')
        self.gt_odom_subscriber = self.create_subscription(Odometry, '/diff_cont/odom', self.gt_odom_callback, 10)

        self.get_logger().info('Subscribing to estimated Odometry topic: /odometry/filtered')
        self.est_odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.est_odom_callback, 10)

        # Initialize variables to store Odometry data
        self.gt_odom_data = None
        self.est_odom_data = None
        self.prev_gt_velocity_x = 0.0
        self.prev_est_velocity_x = 0.0
        self.prev_timestamp = None

    def timer_callback(self):
        try:
            # Get transforms directly
            trans_base_footprint = self.buffer.lookup_transform(
                'odom', 'base_footprint', Time(seconds=0)
            )
            trans_base_footprint_ekf = self.buffer.lookup_transform(
                'odom', 'base_footprint_ekf', Time(seconds=0)
            )

            # Extract ground truth pose data
            gt_pose_data = self.extract_data_from_transform(trans_base_footprint, self.gt_odom_data)

            # Extract estimated pose data
            est_pose_data = self.extract_data_from_transform(trans_base_footprint_ekf, self.est_odom_data)

            # Append ground truth and estimated data
            self.ground_truth_data.append(gt_pose_data)
            self.estimated_data.append(est_pose_data)

            # Publish trajectory markers
            self.publish_trajectory_markers()

            # Check if enough data points collected
            if len(self.ground_truth_data) == 25:  # Assuming you want 50 data points
                # Save data to Excel
                self.save_to_excel()
                # Shut down the node after saving data to Excel
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error('Error getting transforms: {}'.format(str(e)))

    def gt_odom_callback(self, msg: Odometry):
        self.get_logger().info('Received ground truth Odometry message: {}'.format(msg))
        self.gt_odom_data = msg

        if self.prev_timestamp is not None:
            delta_t = (msg.header.stamp.sec - self.prev_timestamp.sec) + \
                      (msg.header.stamp.nanosec - self.prev_timestamp.nanosec) / 1e9
            acceleration_x = (msg.twist.twist.linear.x - self.prev_gt_velocity_x) / delta_t
            acceleration_x = 0.0 if abs(acceleration_x) < 0.0001 else acceleration_x
        else:
            acceleration_x = 0.0

        self.prev_gt_velocity_x = msg.twist.twist.linear.x
        self.prev_timestamp = msg.header.stamp

        self.get_logger().info('Ground Truth Acceleration X: {}'.format(acceleration_x))

    def est_odom_callback(self, msg: Odometry):
        self.get_logger().info('Received estimated Odometry message: {}'.format(msg))
        self.est_odom_data = msg

        if self.prev_timestamp is not None:
            delta_t = (msg.header.stamp.sec - self.prev_timestamp.sec) + \
                      (msg.header.stamp.nanosec - self.prev_timestamp.nanosec) / 1e9
            acceleration_x = (msg.twist.twist.linear.x - self.prev_est_velocity_x) / delta_t
            acceleration_x = 0.0 if abs(acceleration_x) < 0.0001 else acceleration_x
        else:
            acceleration_x = 0.0

        self.prev_est_velocity_x = msg.twist.twist.linear.x
        self.prev_timestamp = msg.header.stamp

        self.get_logger().info('Estimated Acceleration X: {}'.format(acceleration_x))

    def extract_data_from_transform(self, transform, odom_data=None):
        position = transform.transform.translation
        orientation = transform.transform.rotation

        # Convert orientation to yaw
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)

        # Default values for velocity
        velocity_x = 0.0
        velocity_y = 0.0
        angular_velocity_z = 0.0
        acceleration_x=0.0

        if odom_data is not None:
            linear_velocity = odom_data.twist.twist.linear
            angular_velocity = odom_data.twist.twist.angular

            velocity_x = linear_velocity.x
            velocity_y = linear_velocity.y
            angular_velocity_z = angular_velocity.z
            acceleration_x = odom_data.acceleration_x

        return [yaw, velocity_x, velocity_y, angular_velocity_z, acceleration_x,
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w]

    def publish_trajectory_markers(self):
        # Create MarkerArray for ground truth trajectory
        gt_marker_array = MarkerArray()
        gt_marker = Marker()
        gt_marker.header.frame_id = 'odom'
        gt_marker.type = Marker.LINE_STRIP
        gt_marker.action = Marker.ADD
        gt_marker.pose.orientation.w = 1.0
        gt_marker.scale.x = 0.01  # Line width
        gt_marker.color.r = 0.0  # Red color
        gt_marker.color.g = 1.0
        gt_marker.color.b = 0.0
        gt_marker.color.a = 1.0  # Full opacity

        # Add points to ground truth marker
        for data in self.ground_truth_data:
            point = Point()
            point.x = data[5]  # GT_Pos_X
            point.y = data[6]  # GT_Pos_Y
            point.z = data[7]  # GT_Pos_Z
            gt_marker.points.append(point)

        gt_marker_array.markers.append(gt_marker)
        self.publisher_gt_marker.publish(gt_marker_array)

        # Create MarkerArray for estimated trajectory
        est_marker_array = MarkerArray()
        est_marker = Marker()
        est_marker.header.frame_id = 'odom'
        est_marker.type = Marker.LINE_STRIP
        est_marker.action = Marker.ADD
        est_marker.pose.orientation.w = 1.0
        est_marker.scale.x = 0.01  # Line width
        est_marker.color.r = 1.0  # Red color
        est_marker.color.g = 0.0
        est_marker.color.b = 0.0
        est_marker.color.a = 1.0  # Full opacity

        # Add points to estimated marker
        for data in self.estimated_data:
            point = Point()
            point.x = data[5]  # Est_Pos_X
            point.y = data[6]  # Est_Pos_Y
            point.z = data[7]  # Est_Pos_Z
            est_marker.points.append(point)

        est_marker_array.markers.append(est_marker)
        self.publisher_est_marker.publish(est_marker_array)

    def save_to_excel(self):
        excel_filename = 'pose_and_data.xlsx'

        # Create DataFrame for ground truth data
        ground_truth_df = pd.DataFrame(self.ground_truth_data, columns=['GT_Yaw', 'GT_Vel_X', 'GT_Vel_Y', 'GT_Angular_Vel_Z', 'GT_Accel_X',
                                                                        'GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z',
                                                                        'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

        # Create DataFrame for estimated data
        estimated_df = pd.DataFrame(self.estimated_data, columns=['Est_Yaw', 'Est_Vel_X', 'Est_Vel_Y', 'Est_Angular_Vel_Z', 'Est_Accel_X',
                                                                  'Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z',
                                                                  'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

        # Combine ground truth and estimated DataFrames
        combined_df = pd.concat([ground_truth_df, estimated_df], axis=1)

        # Save DataFrame to Excel
        combined_df.to_excel(excel_filename, index=False)

        self.get_logger().info('Data saved to {}'.format(excel_filename))

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

if __name__ == "__main__":
    main()











# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import rclpy.qos
# from sensor_msgs.msg import Imu
# from rclpy.qos import QoSProfile
# from nav_msgs.msg import Odometry 

# class TwistExtractor(Node):
#     def __init__(self):
#         super().__init__('twist_extractor')

#         # Subscribe to /odometry/filtered topic
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/odometry/filtered',
#             self.odom_callback,
#             qos_profile=rclpy.qos.QoSProfile(depth=10)  # Adjust QoS as needed
#         )
#         self.diff_odom_sub = self.create_subscription(
#             Odometry,
#             '/diff_cont/odom',
#             self.diff_odom_callback,
#             qos_profile=rclpy.qos.QoSProfile(depth=10)  # Adjust QoS as needed
#         )

#     def odom_callback(self, msg):
#         """Callback function for /odometry/filtered topic"""
#         linear_x = msg.twist.twist.linear.x
#         linear_y = msg.twist.twist.linear.y
#         linear_z = msg.twist.twist.linear.z
#         angular_x = msg.twist.twist.angular.x
#         angular_y = msg.twist.twist.angular.y
#         angular_z = msg.twist.twist.angular.z

#         # Print the twist message details
#         self.get_logger().info("Odometry (/odometry/filtered):")
#         self.get_logger().info(f"  Linear velocity (x, y, z): {linear_x}, {linear_y}, {linear_z}")

#         # self.get_logger().info("  Angular velocity (x, y, z):", angular_x, angular_y, angular_z)

#     def diff_odom_callback(self, msg):
#         """Callback function for /diff_cont/odom topic"""
#         linear_x = msg.twist.twist.linear.x
#         linear_y = msg.twist.twist.linear.y
#         linear_z = msg.twist.twist.linear.z
#         angular_x = msg.twist.twist.angular.x
#         angular_y = msg.twist.twist.angular.y
#         angular_z = msg.twist.twist.angular.z

#         # Print the twist message details
#         self.get_logger().info("Odometry (/diff_cont/odom):")
#         self.get_logger().info(f"  Linear velocity (x, y, z): {linear_x}, {linear_y}, {linear_z}")

#         # self.get_logger().info("  Angular velocity (x, y, z):", angular_x, angular_y, angular_z)

# def main(args=None):
#     rclpy.init(args=args)

#     twist_extractor = TwistExtractor()
#     rclpy.spin(twist_extractor)

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# import yaml
# import pandas as pd
# from rclpy.node import Node
# from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
# from nav_msgs.msg import Odometry  # Import Odometry message type
# from visualization_msgs.msg import Marker, MarkerArray
# import tf2_ros
# from tf2_geometry_msgs import PointStamped
# import numpy as np
# from rclpy.time import Time
# from tf_transformations import euler_from_quaternion

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')
#         self.get_logger().info('Subscriber node initialized')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_data = []  # Store ground truth data
#         self.estimated_data = []  # Store estimated data

#         # Publishers for markers
#         self.publisher_gt_marker = self.create_publisher(MarkerArray, 'ground_truth_trajectory', 10)
#         self.publisher_est_marker = self.create_publisher(MarkerArray, 'estimated_trajectory', 10)

#         # Subscribers for Odometry messages
#         self.get_logger().info('Subscribing to ground truth Odometry topic: /diff_cont/odom')
#         self.gt_odom_subscriber = self.create_subscription(Odometry, '/diff_cont/odom', self.gt_odom_callback, 10)

#         self.get_logger().info('Subscribing to estimated Odometry topic: /odometry/filtered')
#         self.est_odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.est_odom_callback, 10)

#         # Initialize variables to store Odometry data
#         self.gt_odom_data = None
#         self.est_odom_data = None

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'odom', 'base_footprint', Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'odom', 'base_footprint_ekf', Time(seconds=0)
#             )

#             # Extract ground truth pose data
#             gt_pose_data = self.extract_data_from_transform(trans_base_footprint, self.gt_odom_data)

#             # Extract estimated pose data
#             est_pose_data = self.extract_data_from_transform(trans_base_footprint_ekf, self.est_odom_data)

#             # Append ground truth and estimated data
#             self.ground_truth_data.append(gt_pose_data)
#             self.estimated_data.append(est_pose_data)

#             # Publish trajectory markers
#             self.publish_trajectory_markers()

#             # Check if enough data points collected
#             if len(self.ground_truth_data) == 25:  # Assuming you want 50 data points
#                 # Save data to Excel
#                 self.save_to_excel()
#                 # Shut down the node after saving data to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def gt_odom_callback(self, msg: Odometry):
#         self.get_logger().info('Received ground truth Odometry message: {}'.format(msg))
#         self.gt_odom_data = msg
#         # Print out linear and angular velocities
#         self.get_logger().info('Linear Velocity X: {}'.format(msg.twist.twist.linear.x))
#         self.get_logger().info('Linear Velocity Y: {}'.format(msg.twist.twist.linear.y))
#         self.get_logger().info('Angular Velocity Z: {}'.format(msg.twist.twist.angular.z))

#     def est_odom_callback(self, msg: Odometry):
#         self.get_logger().info('Received estimated Odometry message: {}'.format(msg))
#         self.est_odom_data = msg
#         # Print out linear and angular velocities
#         self.get_logger().info('Linear Velocity X: {}'.format(msg.twist.twist.linear.x))
#         self.get_logger().info('Linear Velocity Y: {}'.format(msg.twist.twist.linear.y))
#         self.get_logger().info('Angular Velocity Z: {}'.format(msg.twist.twist.angular.z))

#     def extract_data_from_transform(self, transform, odom_data=None):
#         position = transform.transform.translation
#         orientation = transform.transform.rotation

#         # Convert orientation to yaw
#         quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
#         _, _, yaw = euler_from_quaternion(quaternion)

#         # Default values for velocity and acceleration
#         velocity_x = 0.0
#         velocity_y = 0.0
#         angular_velocity_z = 0.0
#         acceleration_x = 0.0

#         if odom_data is not None:
#             linear_velocity = odom_data.twist.twist.linear
#             angular_velocity = odom_data.twist.twist.angular

#             velocity_x = linear_velocity.x
#             velocity_y = linear_velocity.y
#             angular_velocity_z = angular_velocity.z

#         return [yaw, velocity_x, velocity_y, angular_velocity_z, acceleration_x,
#                 position.x, position.y, position.z,
#                 orientation.x, orientation.y, orientation.z, orientation.w]

#     def publish_trajectory_markers(self):
#         # Create MarkerArray for ground truth trajectory
#         gt_marker_array = MarkerArray()
#         gt_marker = Marker()
#         gt_marker.header.frame_id = 'odom'
#         gt_marker.type = Marker.LINE_STRIP
#         gt_marker.action = Marker.ADD
#         gt_marker.pose.orientation.w = 1.0
#         gt_marker.scale.x = 0.01  # Line width
#         gt_marker.color.r = 0.0  # Red color
#         gt_marker.color.g = 1.0
#         gt_marker.color.b = 0.0
#         gt_marker.color.a = 1.0  # Full opacity

#         # Add points to ground truth marker
#         for data in self.ground_truth_data:
#             point = Point()
#             point.x = data[5]  # GT_Pos_X
#             point.y = data[6]  # GT_Pos_Y
#             point.z = data[7]  # GT_Pos_Z
#             gt_marker.points.append(point)

#         gt_marker_array.markers.append(gt_marker)
#         self.publisher_gt_marker.publish(gt_marker_array)

#         # Create MarkerArray for estimated trajectory
#         est_marker_array = MarkerArray()
#         est_marker = Marker()
#         est_marker.header.frame_id = 'odom'
#         est_marker.type = Marker.LINE_STRIP
#         est_marker.action = Marker.ADD
#         est_marker.pose.orientation.w = 1.0
#         est_marker.scale.x = 0.01  # Line width
#         est_marker.color.r = 1.0  # Red color
#         est_marker.color.g = 0.0
#         est_marker.color.b = 0.0
#         est_marker.color.a = 1.0  # Full opacity

#         # Add points to estimated marker
#         for data in self.estimated_data:
#             point = Point()
#             point.x = data[5]  # Est_Pos_X
#             point.y = data[6]  # Est_Pos_Y
#             point.z = data[7]  # Est_Pos_Z
#             est_marker.points.append(point)

#         est_marker_array.markers.append(est_marker)
#         self.publisher_est_marker.publish(est_marker_array)

#     def save_to_excel(self):
#         excel_filename = 'pose_and_data.xlsx'

#         # Create DataFrame for ground truth data
#         ground_truth_df = pd.DataFrame(self.ground_truth_data, columns=['GT_Yaw', 'GT_Vel_X', 'GT_Vel_Y', 'GT_Angular_Vel_Z', 'GT_Accel_X',
#                                                                         'GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z',
#                                                                         'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         # Create DataFrame for estimated data
#         estimated_df = pd.DataFrame(self.estimated_data, columns=['Est_Yaw', 'Est_Vel_X', 'Est_Vel_Y', 'Est_Angular_Vel_Z', 'Est_Accel_X',
#                                                                   'Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z',
#                                                                   'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Combine ground truth and estimated DataFrames
#         combined_df = pd.concat([ground_truth_df, estimated_df], axis=1)

#         # Save DataFrame to Excel
#         combined_df.to_excel(excel_filename, index=False)

#         self.get_logger().info('Data saved to {}'.format(excel_filename))

# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped, Twist
# from tf2_ros import Buffer, TransformListener
# from rclpy.time import Time

# class TwistExtractor(Node):

#     def __init__(self):
#         super().__init__('twist_extractor')

#         self.buffer = Buffer()
#         self.listener = TransformListener(self.buffer, self)

#     def extract_twist(self, frame_id):
#         try:
#             # Get transform for the specified frame
#             transform = self.buffer.lookup_transform(
#                 'odom', frame_id, Time(seconds=0)
#             )

#             # Check if 'twist' attribute is available
#             if hasattr(transform, "transform"):
#                 translation = transform.transform.translation
#                 rotation = transform.transform.rotation
#                 self.get_logger().info("Twist extracted for frame: {}".format(frame_id))
#                 self.get_logger().info("Translation: x: {}, y: {}, z: {}".format(translation.x, translation.y, translation.z))
#                 self.get_logger().info("Rotation: x: {}, y: {}, z: {}, w: {}".format(rotation.x, rotation.y, rotation.z, rotation.w))
#             else:
#                 self.get_logger().warn("Transform attribute not found for frame: {}".format(frame_id))

#         except Exception as e:
#             self.get_logger().error("Error getting transform for frame: {} - {}".format(frame_id, str(e)))

# def main(args=None):
#     rclpy.init(args=args)

#     node = TwistExtractor()

#     # Extract twist for base_link
#     node.extract_twist("base_link")

#     # Extract twist for base_footprint_ekf
#     node.extract_twist("base_footprint_ekf")

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped
# import rclpy.time
# import tf2_ros
# from rclpy.time import Time

# class VelocityAccelerationExtractor(Node):

#     def __init__(self):
#         super().__init__('velocity_acceleration_extractor')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.previous_time = None
#         self.previous_translation_x_bl = None
#         self.previous_translation_y_bl = None
#         self.previous_velocity_x_bl = None
#         self.previous_translation_x_ekf = None
#         self.previous_translation_y_ekf = None
#         self.previous_velocity_x_ekf = None
#         self.timer = self.create_timer(2, self.timer_callback)

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_link = self.buffer.lookup_transform(
#                 'odom', 'base_link', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'odom', 'base_footprint_ekf', rclpy.time.Time(seconds=0)
#             )

#             # Extract velocity and acceleration from base_link
#             velocity_x_bl, velocity_y_bl, acceleration_x_bl = self.extract_velocity_acceleration(trans_base_link, 'base_link')

#             # Extract velocity and acceleration from base_footprint_ekf
#             velocity_x_ekf, velocity_y_ekf, acceleration_x_ekf = self.extract_velocity_acceleration(trans_base_footprint_ekf, 'base_footprint_ekf')

#             # Print or process the extracted velocities and accelerations
#             self.get_logger().info("Base link velocity in x: {}".format(velocity_x_bl))
#             self.get_logger().info("Base link velocity in y: {}".format(velocity_y_bl))
#             self.get_logger().info("Base link acceleration in x: {}".format(acceleration_x_bl))

#             self.get_logger().info("Base footprint EKF velocity in x: {}".format(velocity_x_ekf))
#             self.get_logger().info("Base footprint EKF velocity in y: {}".format(velocity_y_ekf))
#             self.get_logger().info("Base footprint EKF acceleration in x: {}".format(acceleration_x_ekf))

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def extract_velocity_acceleration(self, transform, frame_id):
#         velocity_x = 0.0
#         velocity_y = 0.0
#         acceleration_x = 0.0

#         if hasattr(transform, 'header') and hasattr(transform.header, 'stamp') and hasattr(transform.transform, 'translation'):
#             current_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
#             translation_x = transform.transform.translation.x
#             translation_y = transform.transform.translation.y

#             if frame_id == 'base_link':
#                 if self.previous_time is not None:
#                     delta_time = current_time - self.previous_time
#                     delta_translation_x = translation_x - self.previous_translation_x_bl if self.previous_translation_x_bl is not None else 0.0
#                     delta_translation_y = translation_y - self.previous_translation_y_bl if self.previous_translation_y_bl is not None else 0.0
#                     velocity_x = delta_translation_x / delta_time
#                     velocity_y = delta_translation_y / delta_time
#                     if self.previous_velocity_x_bl is not None:
#                         delta_velocity_x = velocity_x - self.previous_velocity_x_bl
#                         acceleration_x = delta_velocity_x / delta_time if delta_time != 0 else 0.0

#                 # Update previous values for next iteration
#                 self.previous_translation_x_bl = translation_x
#                 self.previous_translation_y_bl = translation_y
#                 self.previous_velocity_x_bl = velocity_x

#             elif frame_id == 'base_footprint_ekf':
#                 if self.previous_time is not None:
#                     delta_time = current_time - self.previous_time
#                     delta_translation_x = translation_x - self.previous_translation_x_ekf if self.previous_translation_x_ekf is not None else 0.0
#                     delta_translation_y = translation_y - self.previous_translation_y_ekf if self.previous_translation_y_ekf is not None else 0.0
#                     velocity_x = delta_translation_x / delta_time
#                     velocity_y = delta_translation_y / delta_time
#                     if self.previous_velocity_x_ekf is not None:
#                         delta_velocity_x = velocity_x - self.previous_velocity_x_ekf
#                         acceleration_x = delta_velocity_x / delta_time if delta_time != 0 else 0.0

#                 # Update previous values for next iteration
#                 self.previous_translation_x_ekf = translation_x
#                 self.previous_translation_y_ekf = translation_y
#                 self.previous_velocity_x_ekf = velocity_x

#             self.previous_time = current_time

#         return velocity_x, velocity_y, acceleration_x

# def main(args=None):
#     rclpy.init(args=args)
#     velocity_acceleration_extractor = VelocityAccelerationExtractor()
#     rclpy.spin(velocity_acceleration_extractor)
#     velocity_acceleration_extractor.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# import os
# import rclpy
# import yaml
# import pandas as pd
# from rclpy.node import Node
# from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
# from visualization_msgs.msg import Marker, MarkerArray
# import tf2_ros
# from tf2_geometry_msgs import PointStamped
# import numpy as np
# from rclpy.time import Time
# from tf_transformations import euler_from_quaternion

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_data = []  # Store ground truth data
#         self.estimated_data = []  # Store estimated data

#         # Publishers for markers
#         self.publisher_gt_marker = self.create_publisher(MarkerArray, 'ground_truth_trajectory', 10)
#         self.publisher_est_marker = self.create_publisher(MarkerArray, 'estimated_trajectory', 10)

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'odom', 'base_footprint', Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'odom', 'base_footprint_ekf', Time(seconds=0)
#             )

#             # Extract ground truth pose data
#             gt_data = self.extract_data_from_transform(trans_base_footprint)

#             # Extract estimated pose data
#             est_data = self.extract_data_from_transform(trans_base_footprint_ekf)

#             # Append ground truth and estimated data
#             self.ground_truth_data.append(gt_data)
#             self.estimated_data.append(est_data)

#             # Publish trajectory markers
#             self.publish_trajectory_markers()

#             # Check if enough data points collected
#             if len(self.ground_truth_data) == 20:  # Assuming you want 50 data points
#                 # Save data to Excel
#                 self.save_to_excel()
#                 # Shut down the node after saving data to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def extract_data_from_transform(self, transform, previous_transform=None, time_delta=None):
#         position = transform.transform.translation
#         orientation = transform.transform.rotation

#         # Convert orientation to yaw
#         quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
#         _, _, yaw = euler_from_quaternion(quaternion)

#         # Default values for velocity and acceleration
#         velocity_x = 0.0
#         velocity_y = 0.0
#         angular_velocity_z = 0.0
#         acceleration_x = 0.0

#         # Check if 'twist' attribute is available
#         if hasattr(transform, "twist"):
#             print("Twist attribute found")
#             twist = transform.twist
#             print("Twist message:", twist)
#             linear_velocity = twist.linear
#             angular_velocity = twist.angular
#             print("Linear velocity:", linear_velocity)
#             print("Angular velocity:", angular_velocity)

#             # Extract velocity along x and y
#             velocity_x = linear_velocity.x
#             velocity_y = linear_velocity.y

#             # Extract angular velocity about z-axis
#             angular_velocity_z = angular_velocity.z

#         # Calculate acceleration in x for current frame
#         if previous_transform is not None and time_delta is not None:
#             previous_velocity_x = previous_transform.twist.linear.x
#             acceleration_x = (velocity_x - previous_velocity_x) / time_delta
        
#         return [yaw, velocity_x, velocity_y, angular_velocity_z,acceleration_x,
#                 position.x, position.y, position.z,
#                 orientation.x, orientation.y, orientation.z, orientation.w]

#     def publish_trajectory_markers(self):
#         # Create MarkerArray for ground truth trajectory
#         gt_marker_array = MarkerArray()
#         gt_marker = Marker()
#         gt_marker.header.frame_id = 'odom'
#         gt_marker.type = Marker.LINE_STRIP
#         gt_marker.action = Marker.ADD
#         gt_marker.pose.orientation.w = 1.0
#         gt_marker.scale.x = 0.01  # Line width
#         gt_marker.color.r = 0.0  # Red color
#         gt_marker.color.g = 1.0
#         gt_marker.color.b = 0.0
#         gt_marker.color.a = 1.0  # Full opacity

#         # Add points to ground truth marker
#         for data in self.ground_truth_data:
#             point = Point()
#             point.x = data[5]  # GT_Pos_X
#             point.y = data[6]  # GT_Pos_Y
#             point.z = data[7]  # GT_Pos_Z
#             gt_marker.points.append(point)

#         gt_marker_array.markers.append(gt_marker)
#         self.publisher_gt_marker.publish(gt_marker_array)

#         # Create MarkerArray for estimated trajectory
#         est_marker_array = MarkerArray()
#         est_marker = Marker()
#         est_marker.header.frame_id = 'odom'
#         est_marker.type = Marker.LINE_STRIP
#         est_marker.action = Marker.ADD
#         est_marker.pose.orientation.w = 1.0
#         est_marker.scale.x = 0.01  # Line width
#         est_marker.color.r = 1.0  # Red color
#         est_marker.color.g = 0.0
#         est_marker.color.b = 0.0
#         est_marker.color.a = 1.0  # Full opacity

#         # Add points to estimated marker
#         for data in self.estimated_data:
#             point = Point()
#             point.x = data[5]  # Est_Pos_X
#             point.y = data[6]  # Est_Pos_Y
#             point.z = data[7]  # Est_Pos_Z
#             est_marker.points.append(point)

#         est_marker_array.markers.append(est_marker)
#         self.publisher_est_marker.publish(est_marker_array)

#     def save_to_excel(self):
#         excel_filename = 'pose_and_data.xlsx'

#         # Create DataFrame for ground truth data
#         ground_truth_df = pd.DataFrame(self.ground_truth_data, columns=['GT_Yaw', 'GT_Vel_X', 'GT_Vel_Y', 'GT_Angular_Vel_Z', 'GT_Accel_X',
#                                                                         'GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z',
#                                                                         'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         # Create DataFrame for estimated data
#         estimated_df = pd.DataFrame(self.estimated_data, columns=['Est_Yaw', 'Est_Vel_X', 'Est_Vel_Y', 'Est_Angular_Vel_Z', 'Est_Accel_X',
#                                                                   'Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z',
#                                                                   'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Combine ground truth and estimated DataFrames
#         combined_df = pd.concat([ground_truth_df, estimated_df], axis=1)

#         # Save DataFrame to Excel
#         combined_df.to_excel(excel_filename, index=False)

#         self.get_logger().info('Data saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()



# import os
# import rclpy
# import yaml
# import pandas as pd
# from rclpy.node import Node
# from geometry_msgs.msg import Pose, Point
# import tf2_ros
# from tf2_geometry_msgs import PointStamped
# import numpy as np
# from rclpy.time import Time
# from tf_transformations import euler_from_quaternion
# from geometry_msgs.msg import Twist

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_data = []  # Store ground truth data
#         self.estimated_data = []  # Store estimated data

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'odom', 'base_footprint', Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'odom', 'base_footprint_ekf', Time(seconds=0)
#             )

#             # Extract ground truth pose data
#             gt_data = self.extract_data_from_transform(trans_base_footprint)

#             # Extract estimated pose data
#             est_data = self.extract_data_from_transform(trans_base_footprint_ekf)

#             # Append ground truth and estimated data
#             self.ground_truth_data.append(gt_data)
#             self.estimated_data.append(est_data)

#             # Check if enough data points collected
#             if len(self.ground_truth_data) == 50:  # Assuming you want 50 data points
#                 # Save data to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving data to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def extract_data_from_transform(self, transform):
#         position = transform.transform.translation
#         orientation = transform.transform.rotation

#         # Convert orientation to yaw
#         quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
#         _, _, yaw = euler_from_quaternion(quaternion)

#         # Default values for velocity and acceleration
#         velocity_x = 0.0
#         velocity_y = 0.0
#         angular_velocity_z = 0.0
#         acceleration_x = 0.0

#         # Check if 'twist' attribute is available
#         if hasattr(transform, 'twist'):
#             twist = transform.twist
#             angular_velocity = twist.angular
#             linear_velocity = twist.linear

#             # Extract velocity along x, y
#             velocity_x = linear_velocity.x
#             velocity_y = linear_velocity.y

#             # Extract angular velocity along zif hasattr(transform, 'twist'):
#             twist = transform.twist
#             angular_velocity = twist.angular
#             linear_velocity = twist.linear

#             # Extract velocity along x, y
#             velocity_x = linear_velocity.x
#             velocity_y = linear_velocity.y

#             # Extract angular velocity along z
#             angular_velocity_z = angular_velocity.z

#             # Extract acceleration along x
#             acceleration_x = twist.acceleration.x
#             angular_velocity_z = angular_velocity.z

#             # Extract acceleration along x
#             acceleration_x = twist.acceleration.x

#         return [yaw, velocity_x, velocity_y, angular_velocity_z, acceleration_x,position.x, position.y, position.z,orientation.x, orientation.y, orientation.z, orientation.w]

#     def save_to_excel(self):
#         excel_filename = 'pose_and_data.xlsx'

#         # Create DataFrame for ground truth data
#         ground_truth_df = pd.DataFrame(self.ground_truth_data, columns=['GT_Yaw', 'GT_Vel_X', 'GT_Vel_Y', 'GT_Angular_Vel_Z', 'GT_Accel_X',
#                                                                         'GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z',
#                                                                         'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         # Create DataFrame for estimated data
#         estimated_df = pd.DataFrame(self.estimated_data, columns=['Est_Yaw', 'Est_Vel_X', 'Est_Vel_Y', 'Est_Angular_Vel_Z', 'Est_Accel_X',
#                                                                   'Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z',
#                                                                   'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Combine ground truth and estimated DataFrames
#         combined_df = pd.concat([ground_truth_df, estimated_df], axis=1)

#         # Save DataFrame to Excel
#         combined_df.to_excel(excel_filename, index=False)

#         self.get_logger().info('Data saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()





# import os
# import rclpy
# import tf2_ros
# import yaml
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel
# import matplotlib.pyplot as plt

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.NIS_values = []  # Store NIS values
#         self.Mahalanobis_distances = []  # Store Mahalanobis distances

#         # Load process noise covariance matrix from ekf.yaml
#         self.process_noise_covariance = self.load_process_noise_covariance_from_yaml()

#     def load_process_noise_covariance_from_yaml(self):
#         try:
#             # Get the path to the ekf.yaml file
#             package_name = 'asitlorbot_five_localization'
#             config_file_path = f'/home/asimkumar/asitlor_ws/src/{package_name}/config/ekf.yaml'

#             # Check if the file exists before attempting to open
#             if not os.path.isfile(config_file_path):
#                 self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: File not found at {os.path.abspath(config_file_path)}')
#                 return None

#             # Load the YAML file
#             with open(config_file_path, 'r') as file:
#                 yaml_data = yaml.safe_load(file)

#             # Extract the process noise covariance from the loaded YAML data
#             process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

#             if process_noise_covariance is not None:
#                 # Convert the list elements to float
#                 process_noise_covariance = [float(val) for val in process_noise_covariance]

#                 # Convert the 1D array to a 15x15 matrix
#                 process_noise_covariance_matrix = np.array(process_noise_covariance).reshape((15, 15))

#                 # Ensure the matrix is symmetric (if required)
#                 process_noise_covariance_matrix = 0.5 * (process_noise_covariance_matrix + process_noise_covariance_matrix.T)

#                 return process_noise_covariance_matrix
#             else:
#                 self.get_logger().error('Error loading process_noise_covariance from ekf.yaml: Missing or invalid data.')
#                 return None
#         except Exception as e:
#             self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: {str(e)}')
#             return None

#     def calculate_NIS(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)
#             print(pose_diff)
#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]
#             print(covariance_matrix)
#             # Compute the NIS value and use item() to convert to scalar
#             NIS = (pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff).item()
            
#             return NIS
#         except Exception as e:
#             self.get_logger().error(f'Error calculating NIS: {str(e)}')
#             return None

#     def calculate_Mahalanobis_distance(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             # Compute the Mahalanobis distance
#             mahalanobis_distance = np.sqrt(pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff)

#             return mahalanobis_distance.item()
#         except Exception as e:
#             self.get_logger().error(f'Error calculating Mahalanobis distance: {str(e)}')
#             return None

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Compute and store NIS value
#             NIS_value = self.calculate_NIS(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.NIS_values.append(NIS_value)

#             # Compute and store Mahalanobis distance
#             mahalanobis_distance = self.calculate_Mahalanobis_distance(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.Mahalanobis_distances.append(mahalanobis_distance)

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 50:  # Assuming you want 25 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses, NIS values, and Mahalanobis distances to Excel
#                 self.save_to_excel()

#                 # Plot Mahalanobis distances
#                 self.plot_mahalanobis_distances()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_and_nis_and_mahalanobis_values12_1_11.xlsx' ###############################

#         # Convert poses to DataFrame
#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.ground_truth_poses],
#                                     columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                 columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save NIS values, Mahalanobis distances, and covariance matrix to DataFrames
#         nis_df = pd.DataFrame(self.NIS_values, columns=['NIS'])
#         mahalanobis_df = pd.DataFrame(self.Mahalanobis_distances, columns=['Mahalanobis_Distance'])
#         covariance_df = pd.DataFrame(self.process_noise_covariance.flatten()).T

#         # Combine all DataFrames into a single DataFrame
#         combined_df = pd.concat([ground_truth_df, estimated_df, nis_df, mahalanobis_df, covariance_df], axis=1)

#         # Save DataFrame to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             combined_df.to_excel(writer, sheet_name='Combined_Data', index=False)

#         self.get_logger().info('Combined data saved to {}'.format(excel_filename))


#     def plot_mahalanobis_distances(self):
#         # Plot Mahalanobis distances
#         plt.plot(range(1, len(self.NIS_values) + 1), self.NIS_values, label='NEES')
        
#         # Plot a line for NIS = 9.488
#         plt.axhline(y=9.488, color='r', linestyle='--', label='NEES = 9.488')

#         plt.xlabel('Number of Cycles')
#         plt.ylabel('NEES')
#         plt.title('NEES over Cycles')
#         plt.legend()
#         plt.grid(True)
#         plt.show()


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()











## NIS and Mahalanobis distance
# import os
# import rclpy
# import tf2_ros
# import yaml
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.NIS_values = []  # Store NIS values
#         self.Mahalanobis_distances = []  # Store Mahalanobis distances

#         # Load process noise covariance matrix from ekf.yaml
#         self.process_noise_covariance = self.load_process_noise_covariance_from_yaml()

#     def load_process_noise_covariance_from_yaml(self):
#         try:
#             # Get the path to the ekf.yaml file
#             package_name = 'asitlorbot_five_localization'
#             config_file_path = f'/home/asimkumar/asitlor_ws/src/{package_name}/config/ekf.yaml'

#             # Check if the file exists before attempting to open
#             if not os.path.isfile(config_file_path):
#                 self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: File not found at {os.path.abspath(config_file_path)}')
#                 return None

#             # Load the YAML file
#             with open(config_file_path, 'r') as file:
#                 yaml_data = yaml.safe_load(file)

#             # Extract the process noise covariance from the loaded YAML data
#             process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

#             if process_noise_covariance is not None:
#                 # Convert the list elements to float
#                 process_noise_covariance = [float(val) for val in process_noise_covariance]

#                 # Convert the 1D array to a 15x15 matrix
#                 process_noise_covariance_matrix = np.array(process_noise_covariance).reshape((15, 15))

#                 # Ensure the matrix is symmetric (if required)
#                 process_noise_covariance_matrix = 0.5 * (process_noise_covariance_matrix + process_noise_covariance_matrix.T)

#                 return process_noise_covariance_matrix
#             else:
#                 self.get_logger().error('Error loading process_noise_covariance from ekf.yaml: Missing or invalid data.')
#                 return None
#         except Exception as e:
#             self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: {str(e)}')
#             return None

#     def calculate_NIS(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             print("Shape of pose_diff before reshape:", pose_diff.shape)

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             print("Shape of pose_diff after reshape:", pose_diff.shape)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             print("Shape of covariance_matrix after reshape:", covariance_matrix.shape)

#             # Compute the NIS value and use item() to convert to scalar
#             NIS = (pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff).item()

#             return NIS
#         except Exception as e:
#             self.get_logger().error(f'Error calculating NIS: {str(e)}')
#             return None
#     def calculate_Mahalanobis_distance(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             # Compute the Mahalanobis distance
#             mahalanobis_distance = np.sqrt(pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff)

#             return mahalanobis_distance.item()
#         except Exception as e:
#             self.get_logger().error(f'Error calculating Mahalanobis distance: {str(e)}')
#             return None

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Compute and store NIS value
#             NIS_value = self.calculate_NIS(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.NIS_values.append(NIS_value)

#             # Compute and store Mahalanobis distance
#             mahalanobis_distance = self.calculate_Mahalanobis_distance(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.Mahalanobis_distances.append(mahalanobis_distance)

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 25:  # Assuming you want 50 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses and NIS values to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_and_nis_and_mahalanobis_values.xlsx'

#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                        for pose in self.ground_truth_poses],
#                                       columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                    columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save NIS values to DataFrame
#         nis_df = pd.DataFrame(self.NIS_values, columns=['NIS'])

#         # Save Mahalanobis distances to DataFrame
#         mahalanobis_df = pd.DataFrame(self.Mahalanobis_distances, columns=['Mahalanobis_Distance'])

#         # Save DataFrames to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
#             estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)
#             nis_df.to_excel(writer, sheet_name='NIS_Values', index=False)
#             mahalanobis_df.to_excel(writer, sheet_name='Mahalanobis_Distances', index=False)

#         self.get_logger().info('Poses, NIS values, and Mahalanobis distances saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()









# STORE THE POSE IN EXCEL

# import os
# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 50:  # Assuming you want 50 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_gf.xlsx'

#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                        for pose in self.ground_truth_poses],
#                                       columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                    columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save DataFrames to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
#             estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)

#         self.get_logger().info('Poses saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()





## NIS only

# import os
# import rclpy
# import tf2_ros
# import yaml
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.NIS_values = []  # Store NIS values

#         # Load process noise covariance matrix from ekf.yaml
#         self.process_noise_covariance = self.load_process_noise_covariance_from_yaml()

#     def load_process_noise_covariance_from_yaml(self):
#         try:
#             # Get the path to the ekf.yaml file
#             package_name = 'asitlorbot_five_localization'
#             config_file_path = f'/home/asimkumar/asitlor_ws/src/{package_name}/config/ekf.yaml'

#             # Check if the file exists before attempting to open
#             if not os.path.isfile(config_file_path):
#                 self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: File not found at {os.path.abspath(config_file_path)}')
#                 return None

#             # Load the YAML file
#             with open(config_file_path, 'r') as file:
#                 yaml_data = yaml.safe_load(file)

#             # Extract the process noise covariance from the loaded YAML data
#             process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

#             if process_noise_covariance is not None:
#                 # Convert the list elements to float
#                 process_noise_covariance = [float(val) for val in process_noise_covariance]

#                 # Convert the 1D array to a 15x15 matrix
#                 process_noise_covariance_matrix = np.array(process_noise_covariance).reshape((15, 15))

#                 # Ensure the matrix is symmetric (if required)
#                 process_noise_covariance_matrix = 0.5 * (process_noise_covariance_matrix + process_noise_covariance_matrix.T)

#                 return process_noise_covariance_matrix
#             else:
#                 self.get_logger().error('Error loading process_noise_covariance from ekf.yaml: Missing or invalid data.')
#                 return None
#         except Exception as e:
#             self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: {str(e)}')
#             return None


#     def calculate_NIS(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             print("Shape of pose_diff before reshape:", pose_diff.shape)

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             print("Shape of pose_diff after reshape:", pose_diff.shape)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             print("Shape of covariance_matrix after reshape:", covariance_matrix.shape)

#             # Compute the NIS value and use item() to convert to scalar
#             NIS = (pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff).item()

#             return NIS
#         except Exception as e:
#             self.get_logger().error(f'Error calculating NIS: {str(e)}')
#             return None


#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Compute and store NIS value
#             NIS_value = self.calculate_NIS(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.NIS_values.append(NIS_value)

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 25:  # Assuming you want 50 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses and NIS values to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_and_nis_values.xlsx'

#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                        for pose in self.ground_truth_poses],
#                                       columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                    columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save NIS values to DataFrame
#         nis_df = pd.DataFrame(self.NIS_values, columns=['NIS'])

#         # Save DataFrames to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
#             estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)
#             nis_df.to_excel(writer, sheet_name='NIS_Values', index=False)

#         self.get_logger().info('Poses and NIS values saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()


########################################

# import os
# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.rpe_trans = []  # Store translation RPE
#         self.rpe_rot = []  # Store rotation RPE

#     def calculate_translation_rpe(self, pose_gt, pose_est):
#         # Extract and convert translation vectors
#         position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
#         position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

#         # Calculate squared Euclidean norm and accumulate for all poses
#         translation_diff = position_gt - position_est
#         squared_norm = np.linalg.norm(translation_diff) ** 2

#         return squared_norm

#     def calculate_rotation_rpe(self, pose_gt, pose_est):
#         # Extract and convert rotations (assuming quaternions)
#         quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
#         quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

#         # Ensure quaternions are normalized
#         quaternion_gt /= np.linalg.norm(quaternion_gt)
#         quaternion_est /= np.linalg.norm(quaternion_est)

#         # Calculate difference in angle using arccos for quaternions
#         angle_diff = np.arccos(
#             np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
#         )
 
#         return angle_diff

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later RPE calculation
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 50:  # Assuming you want 10 poses
#                 # Print out collected poses before calculating RPE
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Continue with RPE calculation
#                 m = len(self.ground_truth_poses)
#                 n = len(self.estimated_poses)

#                 # Calculate translation RPE
#                 rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_trans = np.sqrt(rpe_trans_sum / m)
#                 self.rpe_trans.append(rpe_trans)

#                 # Calculate rotation RPE
#                 rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_rot = np.mean(rpe_rot_sum / n)
#                 self.rpe_rot.append(rpe_rot)

#                 # Save RPE values to a CSV file
#                 self.save_to_csv()

#                 # Shut down the node after saving RPE values
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_csv(self):
#         csv_filename = 'rpe_values.csv'

#         # Check if the CSV file already exists
#         file_exists = os.path.isfile(csv_filename)

#         with open(csv_filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             # If the file doesn't exist, write the header
#             if not file_exists:
#                 writer.writerow(['Frame ID', 'Translation RPE', 'Rotation RPE'])

#             # Write the new RPE values
#             for i, (translation_rpe, rotation_rpe) in enumerate(zip(self.rpe_trans, self.rpe_rot)):
#                 writer.writerow([
#                     f'Frame {i}',
#                     translation_rpe,
#                     rotation_rpe,
#                 ])

#         self.get_logger().info('RPE Values appended to {}'.format(csv_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()






########################

# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# from std_msgs.msg import String
# import time
# import signal

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.rpe_trans = []  # Store translation RPE
#         self.rpe_rot = []  # Store rotation RPE

#     def calculate_translation_rpe(self, pose_gt, pose_est):
#         # Extract and convert translation vectors
#         position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
#         position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

#         # Calculate squared Euclidean norm and accumulate for all poses
#         translation_diff = position_gt - position_est
#         squared_norm = np.linalg.norm(translation_diff) ** 2

#         return squared_norm

#     def calculate_rotation_rpe(self, pose_gt, pose_est):
#         # Extract and convert rotations (assuming quaternions)
#         quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
#         quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

#         # Ensure quaternions are normalized
#         quaternion_gt /= np.linalg.norm(quaternion_gt)
#         quaternion_est /= np.linalg.norm(quaternion_est)

#         # Calculate difference in angle using arccos for quaternions
#         angle_diff = np.arccos(
#             np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
#         )

#         return angle_diff

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later RPE calculation
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 5:  # Assuming you want 10 poses
#                 # Print out collected poses before calculating RPE
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Continue with RPE calculation
#                 m = len(self.ground_truth_poses)
#                 n = len(self.estimated_poses)

#                 # Calculate translation RPE
#                 rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_trans = np.sqrt(rpe_trans_sum / m)
#                 self.rpe_trans.append(rpe_trans)

#                 # Calculate rotation RPE
#                 rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_rot = np.mean(rpe_rot_sum / n)
#                 self.rpe_rot.append(rpe_rot)

#                 # Save RPE values to a CSV file
#                 self.save_to_csv()

#                 # Shut down the node after saving RPE values
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_csv(self):
#         csv_filename = 'rpe_values.csv'
#         with open(csv_filename, mode='w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['Frame ID', 'Translation RPE', 'Rotation RPE'])

#             for i, (translation_rpe, rotation_rpe) in enumerate(zip(self.rpe_trans, self.rpe_rot)):
#                 writer.writerow([
#                     f'Frame {i}',
#                     translation_rpe,
#                     rotation_rpe,
#                 ])

#         self.get_logger().info('RPE Values saved to {}'.format(csv_filename))


# def handler(signum, frame):
#     print(f"Received signal {signum}. Shutting down gracefully...")
#     rclpy.shutdown()

# def main(args=None):
#     signal.signal(signal.SIGINT, handler)

#     rclpy.init(args=args)
#     node = Node('rmse_node')
#     subscriber = node.create_subscription(String, 'topic', lambda msg: print(msg.data), 10)
#     rclpy.spin(node)
#     node.destroy_subscription(subscriber)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

###########################




# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np


# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.rpe_trans = []  # Store translation RPE
#         self.rpe_rot = []  # Store rotation RPE

#     def calculate_translation_rpe(self, pose_gt, pose_est):
#         # Extract and convert translation vectors
#         position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
#         position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

#         # Calculate squared Euclidean norm and accumulate for all poses
#         translation_diff = position_gt - position_est
#         squared_norm = np.linalg.norm(translation_diff) ** 2

#         return squared_norm

#     def calculate_rotation_rpe(self, pose_gt, pose_est):
#         # Extract and convert rotations (assuming quaternions)
#         quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
#         quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

#         # Ensure quaternions are normalized
#         quaternion_gt /= np.linalg.norm(quaternion_gt)
#         quaternion_est /= np.linalg.norm(quaternion_est)

#         # Calculate difference in angle using arccos for quaternions
#         angle_diff = np.arccos(
#             np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
#         )
 
#         return angle_diff

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later RPE calculation
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 5:  # Assuming you want 10 poses
#                 # Print out collected poses before calculating RPE
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Continue with RPE calculation
#                 m = len(self.ground_truth_poses)
#                 n = len(self.estimated_poses)

#                 # Calculate translation RPE
#                 rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_trans = np.sqrt(rpe_trans_sum / m)
#                 self.rpe_trans.append(rpe_trans)

#                 # Calculate rotation RPE
#                 rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_rot = np.mean(rpe_rot_sum / n)
#                 self.rpe_rot.append(rpe_rot)

#                 # Save RPE values to a CSV file
#                 self.save_to_csv()

#                 # Shut down the node after saving RPE values
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_csv(self):
#         csv_filename = 'rpe_values.csv'
#         with open(csv_filename, mode='w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['Frame ID', 'Translation RPE', 'Rotation RPE'])

#             for i, (translation_rpe, rotation_rpe) in enumerate(zip(self.rpe_trans, self.rpe_rot)):
#                 writer.writerow([
#                     f'Frame {i}',
#                     translation_rpe,
#                     rotation_rpe,
#                 ])

#         self.get_logger().info('RPE Values saved to {}'.format(csv_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()
    