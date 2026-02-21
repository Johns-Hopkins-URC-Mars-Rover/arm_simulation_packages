import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("arm_jr", package_name="armjr_moveit_config")
        .to_moveit_configs()
    )

    controllers_yaml = os.path.join(
        get_package_share_directory("armjr_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    return LaunchDescription([

        # REQUIRED: publishes URDF to /robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),

        # REQUIRED: ros2_control hardware + controllers
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                controllers_yaml,
            ],
        ),

        # REQUIRED: spawn controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "velocity_controller",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
        ),

    ])