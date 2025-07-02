from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('stereo_cam')

    # Config files
    raspi_config = os.path.join(pkg_dir, 'config', 'cameras', 'raspi_camera_params.yaml')
    laptop_config = os.path.join(pkg_dir, 'config', 'cameras', 'laptop_camera_params.yaml')

    # Determine which config to use based on 'use_raspi'
    use_raspi = LaunchConfiguration('use_raspi').perform(context)
    selected_config = raspi_config if use_raspi.lower() == 'true' else laptop_config

    # Load URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'stereo_camera.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Composable container for stereo_node
    stereo_container = ComposableNodeContainer(
        name='stereo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='stereo_cam',
                plugin='stereo_cam::StereoNode',
                name='stereo_node',
                parameters=[
                    selected_config,
                    {'robot_description': robot_description},
                    {'enable_depth': LaunchConfiguration('enable_depth')}
                ]
            ),
        ],
        output='screen',
    )

    # Return nodes
    return [
        stereo_container,

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
            default_value='false',
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
