import os
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Dynamically infer the package name based on the launch file location
    launch_file_dir = Path(__file__).resolve().parent
    package_dir = launch_file_dir.parent  # assumes `launch/` is under the package root
    package_name = package_dir.name

    # Get full share directory (to support install/ prefixes)
    package_share_dir = get_package_share_directory(package_name)

    # Load extra parameter YAMLs
    joint_limits_path = Path(package_share_dir) / "config" / "joint_limits.yaml"
    pilz_limits_path = Path(package_share_dir) / "config" / "pilz_cartesian_limits.yaml"

    # with open(joint_limits_path, "r") as f:
    #     joint_limits = yaml.safe_load(f)

    with open(pilz_limits_path, "r") as f:
        pilz_limits = yaml.safe_load(f)

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # Set use_sim_time to true
    set_use_sim_time = SetLaunchConfiguration(
        name="use_sim_time",
        value="true"
    )

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(robot_name="tm12", package_name="tm12_gazebo_moveit_config")
        .joint_limits(joint_limits_path)
        .pilz_cartesian_limits(pilz_limits_path)
        .trajectory_execution()
        .to_moveit_configs()
    )

    common_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.joint_limits,
        pilz_limits,
        {"use_sim_time": True},
    ]

    # Launch RViz2 node manually, with available parameters
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=common_params,
        arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")]
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_params,
    )


    return LaunchDescription([
        use_sim_time_arg,
        set_use_sim_time,
        move_group_node,
        rviz_node
    ])
