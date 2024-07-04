import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart





def generate_launch_description():


    package_name='my_robot'
    
    lidar_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rplidar.launch.py')
    lcd_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'lcd_i2c_16x2.launch.py')
    
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )
    
    lcd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lcd_launch_path)
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(
        get_package_share_directory(package_name), 
        'params',
        'my_real_robot_controllers.yaml'
        )

    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                controller_params],
        )  
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    twist_mux = Node(
        package='custom_twist_mux',
        executable='twist_mux',
        name='custom_twist_mux',
        parameters=[{
            'sources': ['cmd_vel', 'cmd_vel_joy'],
            'priorities': [2, 1],
            'cmd_vel_out' : 'diff_cont/cmd_vel_unstamped'
        }]
    )

    return LaunchDescription([
        lcd,
        twist_mux,
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        lidar,
    ])