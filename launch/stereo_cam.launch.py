from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('stereo_cam')

    # Config files
    raspi_config = os.path.join(pkg_dir, 'config', 'raspi_camera_params.yaml')
    laptop_config = os.path.join(pkg_dir, 'config', 'laptop_camera_params.yaml')

    # Determine which config to use based on 'use_raspi'
    use_raspi = LaunchConfiguration('use_raspi').perform(context)
    selected_config = raspi_config if use_raspi.lower() == 'true' else laptop_config

    # Load URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'stereo_camera.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Return nodes
    return [
        Node(
            package='stereo_cam',
            executable='stereo_node',
            parameters=[
                selected_config,
                {'robot_description': robot_description},
                {'enable_depth': LaunchConfiguration('enable_depth')}
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'stereo_cam.rviz')]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_image_proc'),
                    'launch',
                    'stereo_image_proc.launch.py'
                ])
            ),
            launch_arguments={
                # 'namespace': 'camera'
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_rect'))
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_raspi',
            default_value='false',
            description='Use Raspberry Pi config if true, laptop config otherwise'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='Enable depth estimation node'
        ),
        DeclareLaunchArgument(
            'enable_rect',
            default_value='true',
            description='Enable stereo rectification node'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Whether to start RVIZ'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
