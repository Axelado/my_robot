from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lcd_i2c_my_robot',
            executable='lcd_i2c_16x2',
            name='lcd_i2c_16x2',
        )
    ])