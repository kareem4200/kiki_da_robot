import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    urdf_file = os.path.join(
        get_package_share_directory('kiki_bot'),
        'urdf',
        'kiki.urdf'
    )

    with open(urdf_file, 'r') as f:
        desc = f.read()

    rviz_file = os.path.join(
        get_package_share_directory('kiki_bot'),
        'config',
        'display_urdf.rviz'
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file,
            description='Absolute path to robot URDF file'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': desc}],
            arguments = [urdf_file]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joints',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d' + rviz_file]
        )])
