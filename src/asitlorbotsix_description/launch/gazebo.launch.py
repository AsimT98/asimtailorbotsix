import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    asitlorbotsix_description = get_package_share_directory("asitlorbotsix_description")
    asitlorbotsix_description_prefix = get_package_prefix("asitlorbotsix_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        asitlorbotsix_description, "urdf", "asitlorbotsix.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )
    # x_pose = LaunchConfiguration('x_pose', default='-2.0')
    # y_pose = LaunchConfiguration('y_pose', default='-0.5')
    model_path = os.path.join(asitlorbotsix_description, "models")
    model_path += pathsep + os.path.join(asitlorbotsix_description_prefix, "share")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "asitlorbotsix",
                                   "-topic", "robot_description",
                                #    "-x", x_pose,
                                #     "-y", y_pose
                                  ],
                        output="screen",
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])