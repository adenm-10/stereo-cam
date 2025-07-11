import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():

    stereo_pkg_dir = get_package_share_directory('stereo_cam')
    ekf_mpu_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf_mpu.yaml')
    ekf_mpu_params = ParameterFile(ekf_mpu_config_path)
    ekf_bno_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf_bno.yaml')
    ekf_bno_params = ParameterFile(ekf_bno_config_path)

    mpu_pkg_dir = get_package_share_directory('mpu9250driver')
    mpu_imu_config_path = os.path.join(mpu_pkg_dir, 'params', 'mpu_imu.yaml')
    mpu_imu_params = ParameterFile(mpu_imu_config_path)

    bno_pkg_dir = get_package_share_directory('bno08x_driver')
    bno_imu_config_path = os.path.join(bno_pkg_dir, 'config', 'bno085_i2c.yaml')
    bno_imu_params = ParameterFile(bno_imu_config_path)
    
    use_bno = LaunchConfiguration('use_bno')

    return LaunchDescription([

        DeclareLaunchArgument('use_bno', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='INFO'),

        # Drivers 
        Node(
            package='mpu9250driver',
            executable='mpu9250driver',
            name='mpu9250driver_node',
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[mpu_imu_params],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=UnlessCondition(use_bno)
        ),

        Node(
            package='bno08x_driver',  
            executable='bno08x_driver',  
            name='bno08x_driver',
            output='screen',
            parameters=[bno_imu_params],
            condition=IfCondition(use_bno)
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
            }.items()
        ),
        
        # 3. EKF Node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_bno_params],
            condition=IfCondition(use_bno)
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_mpu_params],
            condition=UnlessCondition(use_bno)
        ),
    ])
