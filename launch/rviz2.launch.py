from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'my_robot'
    
    # Declare the RVIZ config file argument
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory(package_name), 'rviz', 'mapping.rviz'),
        description='Full path to the RVIZ config file to use'
    )
    
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # Define the RVIZ2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Return the launch description
    return LaunchDescription([
        rviz_config_file_arg,
        rviz2
    ])