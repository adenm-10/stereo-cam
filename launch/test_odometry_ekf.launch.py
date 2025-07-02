import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    stereo_pkg_dir = get_package_share_directory('stereo_cam')
    imu_pkg_dir = get_package_share_directory('mpu9250driver')

    ekf_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf.yaml')
    ekf_params = ParameterFile(ekf_config_path)

    mpu_imu_config_path = os.path.join(imu_pkg_dir, 'params', 'mpu_imu.yaml')
    mpu_imu_params = ParameterFile(mpu_imu_config_path)

    return LaunchDescription([

        # Drivers
        Node(
            package='mpu9250driver',
            executable='mpu9250driver',
            name='mpu9250driver_node',
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[mpu_imu_params]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_cam'),
                    'launch',
                    'image_proc_pipeline.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': 'true',
                'enable_disparity': 'false',
            }.items()
        ),

        # 3. EKF Node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),
    ])
