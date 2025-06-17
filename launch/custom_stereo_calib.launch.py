from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('stereo_cam')

    # Read runtime launch configurations
    use_raspi = LaunchConfiguration('use_raspi').perform(context)
    selected_config = os.path.join(pkg_dir, 'config', 'raspi_camera_params.yaml') \
        if use_raspi.lower() == 'true' else \
        os.path.join(pkg_dir, 'config', 'laptop_camera_params.yaml')
    
    print(selected_config)

    # Stereo camera node
    stereo_node = Node(
        package='stereo_cam',
        executable='stereo_node',
        parameters=[
            selected_config,
            {'enable_depth': LaunchConfiguration('enable_depth')}
        ],
        output='screen'
    )

    # Calibration node (delayed)
    calib_node = TimerAction(
        period=3.0,
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

    return [stereo_node, calib_node]


def generate_launch_description():
    # Declare args
    calib_args = [
        DeclareLaunchArgument('num_corners_vertical', default_value='6'),
        DeclareLaunchArgument('num_corners_horizontal', default_value='4'),
        DeclareLaunchArgument('square_size_mm', default_value='30'),
        DeclareLaunchArgument('show_chess_corners', default_value='true'),
        DeclareLaunchArgument('use_raspi', default_value='false'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
    ]

    return LaunchDescription(
        calib_args + [
            OpaqueFunction(function=launch_setup)
        ]
    )
