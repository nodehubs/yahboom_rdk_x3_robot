from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ms200_scan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('oradar_lidar'), 'launch'),
         '/ms200_scan.launch.py'])
      )

    bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_bringup'), 'launch'),
        '/yahboomcar_bringup_launch.py'])
    )
    
    patrol_node = Node(
    	package='yahboomcar_bringup',
    	executable='patrol',
    	output='screen'
    )
    launch_description = LaunchDescription([
        ms200_scan_node,
        bringup_node,
        patrol_node
        ]) 
    return launch_description

