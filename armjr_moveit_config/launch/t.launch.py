import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterFile
from launch import conditions
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Warning: Could not load {file_path}: {e}")
        return {}

def generate_launch_description():

    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "link6",
        "dof": "5",
    }

    # Manually load kinematics.yaml to avoid parameter parsing issues
    kinematics_yaml = load_yaml("armjr_moveit_config", "config/kinematics.yaml")
    print(f"Loaded kinematics config: {kinematics_yaml}")
    
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "arm_jr", package_name="armjr_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]  # Simplified: removed problematic planners
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server - FIXED PARAMETER ORDER
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            #moveit_config.kinematics_yaml,   #Use manually loaded YAML instead of ParameterFile
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
            {"robot_description_kinematics": kinematics_yaml},
        ],
    )

    # RViz for visualization
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("armjr_moveit_config"), "config", rviz_base]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using mock hardware for trajectory execution
    ros2_controllers_path = os.path.join(
        get_package_share_directory("armjr_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        rviz_config_arg,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
        rviz_node,
    ])
