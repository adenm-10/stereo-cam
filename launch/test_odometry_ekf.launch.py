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
                'enable_depth': 'false',
                'enable_rect': 'true',
            }.items()
        ),

        # 2. Stereo visual odometry
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='rtabmap_odom',
            parameters=[rtab_odom_params]
        ),

        # 3. EKF Node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        # # 4. RTAB-Map SLAM node
        # Node(
        #     package='rtabmap_ros',
        #     executable='rtabmap',
        #     name='rtabmap',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'subscribe_odom_info': True,
        #         'odom_topic': '/odometry/filtered',
        #         'use_sim_time': False
        #     }],
        #     remappings=[
        #         ('odom', '/odometry/filtered')
        #     ]
        # )
    ])
