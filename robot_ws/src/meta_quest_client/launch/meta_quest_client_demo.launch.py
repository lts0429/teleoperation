import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    nodes = []

    # meta quest client
    meta_quest_client_node = Node(
        package='meta_quest_client', 
        executable='udp_client', 
        output='screen'
    )
    nodes.append(meta_quest_client_node)
    
    #rviz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('meta_quest_client'),
        'config',
        'standard.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'error'],
        parameters=[{'use_sim_time': False}]
    )
    nodes += [rviz_node]
    
    return LaunchDescription(nodes)
