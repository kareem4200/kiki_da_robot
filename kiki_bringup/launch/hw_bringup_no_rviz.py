from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kiki_description"), "urdf", "kiki.urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kiki_control"),
            "param",
            "diffbot_controllers.yaml",
        ]
    )

    lidar_launch_file = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
        [
            FindPackageShare("ydlidar_ros2_driver"),
            "launch",
            "ydlidar_launch.py",
        ]
    ))

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kiki_control"), "config", "diffbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # remappings=[
        #     ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        # parameters=[{"type": "joint_state_broadcaster/Joi]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
        parameters=[robot_controllers],
        # remappings=[
        # ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),  # remap relative topic
        # ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    lidar_odom_node = Node(
        package="lidar_odometry",
        executable="lidar_odometry_node",
        name="lidar_odometry_node",
        output="screen",
    )

    # delay_odom_after_lidar = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         IncludeLaunchDescription(lidar_launch_file),
    #         on_exit=[lidar_odom_node],
    #     )
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        IncludeLaunchDescription(lidar_launch_file),
        lidar_odom_node,
    ]

    return LaunchDescription(nodes)