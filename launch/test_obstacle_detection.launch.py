import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')

    ekf_config_path = os.path.join(pkg_dir, 'config', 'odom', 'ekf_test.yaml')
    ekf_params = ParameterFile(ekf_config_path)

    rtab_odom_config_path = os.path.join(pkg_dir, 'config', 'odom', 'rtab_odom.yaml')
    rtab_odom_params = ParameterFile(rtab_odom_config_path)

    mpu_imu_config_path = os.path.join(pkg_dir, 'params', 'mpu_imu.yaml')
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
                'use_raspi': 'false',
                'enable_depth': 'false',
                'enable_rect': 'true',
            }.items()
        ),

        # 2. Stereo visual odometry
        Node(
            package='stereo_cam',
            executable='obstacle_detector',
        ),
    ])
