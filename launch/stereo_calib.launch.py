from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')
    
    # Declare calibration parameters
    calib_args = [
        DeclareLaunchArgument(
            'num_corners_vertical',
            default_value='6',
            description='Number of inner corners vertically in the calibration pattern'
        ),
        DeclareLaunchArgument(
            'num_corners_horizontal',
            default_value='4',
            description='Number of inner corners horizontally in the calibration pattern'
        ),
        DeclareLaunchArgument(
            'square_size_mm',
            default_value='30',
            description='Size of each square in the calibration pattern in mm'
        ),
        DeclareLaunchArgument(
            'show_chess_corners',
            default_value='true',
            description='Show detected chess corners during calibration'
        )
    ]

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
                parameters=[os.path.join(pkg_dir, 'config', 'cameras', 'laptop_camera_params.yaml')],
            ),
        ],
        output='screen',
    )

    # Calibration node with delayed start
    calib_node = TimerAction(
        period=3.0,  # 3 second delay
        actions=[
            Node(
                package='stereo_cam',
                executable='stereo_calib',
                parameters=[{
                    'num_corners_vertical': LaunchConfiguration('num_corners_vertical'),
                    'num_corners_horizontal': LaunchConfiguration('num_corners_horizontal'),
                    'square_size_mm': LaunchConfiguration('square_size_mm'),
                    'show_chess_corners': LaunchConfiguration('show_chess_corners')
                }],
                output='screen',
                arguments=['--use-intra-process-comms']
            )
        ]
    )

    return LaunchDescription(
        calib_args + [
            stereo_container,
            calib_node
        ]
    )
