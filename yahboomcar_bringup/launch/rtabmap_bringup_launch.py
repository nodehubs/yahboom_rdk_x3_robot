from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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

    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver',
    )
    
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true',
                                            description='Whether to publish the tf from the original odom to the base_footprint')
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        output = "screen"
    )

    imu_filter_config = os.path.join(
         get_package_share_directory('yahboomcar_bringup'), 
         'params',
         'imu_filter_param.yaml'
      )
    
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )
    
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_localization'), 'launch'),
         '/ekf.launch.py'])
      )
 
    yahboom_joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('yahboomcar_ctrl'), 'launch'),
         '/yahboomcar_joy_launch.py'])
    )
    
    yahboom_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('yahboomcar_description'), 'launch'),
         '/description_launch.py'])
    )

    return LaunchDescription([
        ms200_scan_node,
        driver_node,
        pub_odom_tf_arg,
        base_node,
        # imu_filter_node,
        # ekf_node,
        yahboom_joy_node,
        yahboom_description_node
    ])
