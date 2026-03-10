import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_explorer = get_package_share_directory('autonomous_explorer')

    
    nav2_params = os.path.join(pkg_explorer, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(pkg_explorer, 'config', 'simple.xml')

    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'default_nav_to_pose_bt_xml': bt_xml, 
            'autostart': 'true'
        }.items()
    )


    explorer_node = Node(
        package='autonomous_explorer',
        executable='explorer_node',
        name='explorer_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        nav2_launch,
        explorer_node
    ])