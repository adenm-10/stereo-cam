from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('use_raspi', default_value='true'),
        DeclareLaunchArgument('output_control', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_cam'),
                    'launch',
                    'stereo_cam.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': LaunchConfiguration('use_raspi'),
            }.items()
        ),

        # 2. Stereo Object Detection
        Node(
            package='stereo_cam',
            executable='obstacle_detector',
        ),

        Node(
            package='navis_nav',
            executable='control_out_node',
            condition=IfCondition(LaunchConfiguration('output_control'))
        ),
    ])
