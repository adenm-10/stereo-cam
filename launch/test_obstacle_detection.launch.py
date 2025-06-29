import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')

    ekf_config_path = os.path.join(pkg_dir, 'config', 'odom', 'ekf_test.yaml')
    ekf_params = ParameterFile(ekf_config_path)

    rtab_odom_config_path = os.path.join(pkg_dir, 'config', 'odom', 'rtab_odom.yaml')
    rtab_odom_params = ParameterFile(rtab_odom_config_path)

    mpu_imu_config_path = os.path.join(pkg_dir, 'config', 'odom', 'mpu_imu.yaml')
    mpu_imu_params = ParameterFile(mpu_imu_config_path)

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_cam'),
                    'launch',
                    'stereo_cam.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': 'true',
                'enable_depth': 'false',
                'enable_rect': 'true',
            }.items()
        ),

        # 2. Stereo visual odometry
        Node(
            package='stereo_cam',
            executable='obstacle_detector',
        ),

        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            parameters=[{
                'approximate_sync': True,
                'use_system_default_qos': False,
                'stereo_algorithm': 0,
                'prefilter_size': 9,
                'prefilter_cap': 31,
                'correlation_window_size': 15,
                'min_disparity': 0,
                'disparity_range': 128,
                'texture_threshold': 10,
                'speckle_size': 100,
                'speckle_range': 4,
                'disp12_max_diff': 1,
                'uniqueness_ratio': 15.0,
                'P1': 0.0,
                'P2': 0.0,
                'full_dp': False
            }],
            remappings=[
                # ('left/image_rect', '/olive/camera/olv_cam01/image_rect'), 
                # ('left/camera_info', '/olive/camera/olv_cam01/camera_info_rect'),   
                # ('right/image_rect', '/olive/camera/olv_cam02/image_rect'), 
                # ('right/camera_info','/olive/camera/olv_cam02/camera_info_rect')
            ]
        ),
        
        Node(
            package='stereo_image_proc',
            executable= 'point_cloud_node',
            parameters=[{
                'approximate_sync': True,
                'avoid_point_cloud_padding': False,
                'use_color': True,
                'use_system_default_qos': False,
            }],
            remappings=[
                # ('left/camera_info', '/olive/camera/olv_cam01/camera_info_rect'), 
                # ('right/camera_info', '/olive/camera/olv_cam02/camera_info_rect'), 
                # (
                    # 'left/image_rect_color', '/olive/camera/olv_cam01/image_rect'
                # ),
            ]
        ),
    ])
