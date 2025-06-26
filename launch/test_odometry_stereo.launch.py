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
    rtab_odom_config_path = os.path.join(pkg_dir, 'config', 'odom', 'rtab_odom.yaml')
    rtab_odom_params = ParameterFile(rtab_odom_config_path)

    return LaunchDescription([

        # Drivers
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
    ])
