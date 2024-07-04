import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "my_robot"
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(package_name), 'map', 'yellolab.yaml'),
        description='Full path to the map to use for localization'
    )

    map = LaunchConfiguration('map')
    
    # Construire les chemins des fichiers de lancement
    joystick_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
    localization_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'localization_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    
    # Chemin du fichier de configuration RViz
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'localization.rviz')

    # Imprimer les chemins pour vérifier leur exactitude
    print(f"joystick_launch_path: {joystick_launch_path}")
    print(f"slam_launch_path: {localization_launch_path}")
    print(f"navigation_launch_path: {navigation_launch_path}")
    print(f"rviz_slaunch_path: {rviz_launch_path}")
    
    # Vérifier que les fichiers existent
    if not os.path.isfile(joystick_launch_path):
        print(f"Error: {joystick_launch_path} does not exist")
    if not os.path.isfile(localization_launch_path):
        print(f"Error: {localization_launch_path} does not exist")
    if not os.path.isfile(navigation_launch_path):
        print(f"Error: {navigation_launch_path} does not exist")
    if not os.path.isfile(rviz_launch_path):
        print(f"Error: {rviz_launch_path} does not exist")
    if not os.path.isfile(rviz_config_file):
        print(f"Error: {rviz_config_file} does not exist")
    
    # Inclure les fichiers de lancement    
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_path), 
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path), 
        launch_arguments={'use_sim_time': 'false',
                          'map' : map}.items()
    )
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path), 
        launch_arguments={'use_sim_time': 'false',
                          'map_subscribe_trasient_local': 'true'}.items()
    )
    
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path), 
        launch_arguments={'use_sim_time': 'false',
                          'rviz_config_file': rviz_config_file}.items()
    )
    
    return LaunchDescription([
        map_arg,
        localization,
        navigation,
        joystick,
        rviz2,
    ])
