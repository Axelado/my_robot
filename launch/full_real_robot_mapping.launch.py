import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "my_robot"
    
    # Construire les chemins des fichiers de lancement
    joystick_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
    slam_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'online_async_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    
    # Chemin du fichier de configuration RViz
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'mapping.rviz')

    # Imprimer les chemins pour vérifier leur exactitude
    print(f"joystick_launch_path: {joystick_launch_path}")
    print(f"slam_launch_path: {slam_launch_path}")
    print(f"navigation_launch_path: {navigation_launch_path}")
    print(f"rviz_launch_path: {rviz_launch_path}")
    
    
    # Vérifier que les fichiers existent
    if not os.path.isfile(joystick_launch_path):
        print(f"Error: {joystick_launch_path} does not exist")
    if not os.path.isfile(slam_launch_path):
        print(f"Error: {slam_launch_path} does not exist")
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
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path), 
        launch_arguments={'use_sim_time': 'false'}.items()
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
        slam,
        navigation, 
        joystick,
        rviz2,
    ])
