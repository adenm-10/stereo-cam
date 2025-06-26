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
                    'stereo_cam.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': 'true',
                'enable_depth': 'true',
                'enable_rect': 'false',
            }.items()
        ),

        # 2. Stereo visual odometry
        Node(
            package='rtabmap_odom',
            executable='rtabmap-odom',
            name='rtabmap_odom',
            output='screen',
            parameters=[{
                'subscribe_depth': True,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # let EKF handle it
                'queue_size': 30,
                'approx_sync': True,
                'wait_imu_to_init': False,
            }],
            remappings=[
                ('rgb/image', '/left/image_color'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/left/camera_info'),
            ]
        ),
    ])
