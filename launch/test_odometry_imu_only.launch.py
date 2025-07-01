import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')

    ekf_config_path = os.path.join(pkg_dir, 'config', 'odom', 'ekf_imu_only.yaml')
    ekf_params = ParameterFile(ekf_config_path)

    mpu_imu_config_path = os.path.join(pkg_dir, 'config', 'odom', 'mpu_imu.yaml')
    mpu_imu_params = ParameterFile(mpu_imu_config_path)

    return LaunchDescription([

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

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        Node(
            package='mpu9250driver',
            executable='odom_tracker_node'
        ),
    ])
