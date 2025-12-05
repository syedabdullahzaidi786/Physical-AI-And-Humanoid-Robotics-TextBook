from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Launch robot_state_publisher for the URDF and spawn in Gazebo
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Example placeholder for Gazebo spawn (user must adapt to their Gazebo setup)
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        name='gazebo',
        output='screen',
    )

    return LaunchDescription([
        rsp_node,
        gazebo_node,
    ])
