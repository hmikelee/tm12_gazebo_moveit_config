from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_world = LaunchConfiguration("gz_world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    declare_gz_world = DeclareLaunchArgument(
        "gz_world", default_value="empty.sdf",
        description="Gazebo world file name"
    )

    # Process Xacro
    # Xacro processing
    description_pkg = FindPackageShare("tm12_gazebo_moveit_config")
    xacro_file = PathJoinSubstitution([description_pkg, "config", "tm12.urdf.xacro"])
    robot_description = Command(["xacro ", xacro_file])

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    # Add path to the controller YAML
    controllers_file = PathJoinSubstitution([
        FindPackageShare("tm12_gazebo_moveit_config"),
        "config",
        "tm12_controllers.yaml"
    ])

    # Start ros2_control_node (no plugin in URDF!)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
            controllers_file
        ],
        output="screen"
    )


    # Start Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", gz_world],
        output="screen"
    )

    # Spawn robot in Gazebo via ros_gz_sim create
    spawn_tm12 = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "tm12",
            "-topic", "robot_description"
        ],
        output="screen"
    )


    # Spawn controllers (delayed to ensure controller manager is up)
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster"
        ],
        output="screen"
    )

    
    spawn_tm12_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "tm12_arm_controller"
        ],
        output="screen"
    )


    # Start /clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # (Optional) controller manager and spawners could go here

    return LaunchDescription([
        declare_use_sim_time,
        declare_gz_world,
        rsp_node,
        clock_bridge,
        gz_sim,
        control_node,
        TimerAction(
            period=2.0,
            actions=[spawn_tm12]
        ),
        TimerAction(
            period=4.0,
            actions=[spawn_joint_state_broadcaster, spawn_tm12_arm_controller]
        )
    ])


