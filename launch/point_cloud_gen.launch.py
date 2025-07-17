from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
                    {'enable_depth': 'false'}
                ]
            ),
        ],
        output='screen',
    )

    depth_node = Node(
        package='stereo_cam',
        executable='depth_node'
    )

    # Return nodes
    return [
        stereo_container,
        depth_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_raspi',
            default_value='false',
            description='Use Raspberry Pi config if true, laptop config otherwise'
        ),
        OpaqueFunction(function=launch_setup)
    ])
