import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PythonExpression, Command
)
from launch_ros.actions import (
    ComposableNodeContainer, Node
)
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

# ───────────────────────── helper: build component list ───────────────────
def compose_perception(context):
    pkg_dir = get_package_share_directory('stereo_cam')

    raspi_cfg   = os.path.join(pkg_dir, 'config/cameras/raspi_camera_params.yaml')
    laptop_cfg  = os.path.join(pkg_dir, 'config/cameras/laptop_camera_params.yaml')
    use_raspi   = LaunchConfiguration('use_raspi').perform(context).lower() == 'true'
    cam_cfg     = raspi_cfg if use_raspi else laptop_cfg

    log_level = LaunchConfiguration('log_level')

    rtab_stereo_params = ParameterFile(os.path.join(
        pkg_dir, 'config/odom/rtab_odom_stereo.yaml'
    ))

    rtab_rgbd_params = ParameterFile(os.path.join(
        pkg_dir, 'config/odom/rtab_odom_stereo.yaml'
    ))

    disparity_params = ParameterFile(os.path.join(
        pkg_dir, 'config/disparity.yaml'
    ))

    # build /robot_description once so the same ParameterValue is reused
    urdf_file = os.path.join(pkg_dir, 'urdf', 'stereo_camera.urdf.xacro')
    robot_description = {
        'robot_description': Command(['xacro ', urdf_file])
    }

    # ───────── component list ─────────
    components = [
        # 1. driver
        ComposableNode(
            package='stereo_cam',
            plugin='stereo_cam::StereoNode',
            name='stereo_node',
            parameters=[
                cam_cfg,
                robot_description,
            ],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ]
        ),
        # 2a left rectify
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='left_rectify',
            remappings=[
                ('image',          '/left/image_raw'),
                ('camera_info',    '/left/camera_info'),
                ('image_rect',     '/left/image_rect'),
                ('camera_info_out','/left/camera_info_rect'),
            ],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ]
        ),
        # 2b right rectify
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='right_rectify',
            remappings=[
                ('image',          '/right/image_raw'),
                ('camera_info',    '/right/camera_info'),
                ('image_rect',     '/right/image_rect'),
                ('camera_info_out','/right/camera_info_rect'),
            ],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ]
        ),

        # Extra disparity node for obstacle detection
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            name='disparity_node',
            parameters=[disparity_params],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ],
            condition=IfCondition(LaunchConfiguration('enable_disparity')),
        ),

        # 4a Stereo Visual Odometry
        ComposableNode(
            package='rtabmap_odom',
            plugin='rtabmap_odom::StereoOdometry',
            name='rtabmap_odom',
            parameters=[rtab_stereo_params],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ],
            condition=UnlessCondition(LaunchConfiguration('rgbd_mode'))
        ),

        # 4b RGBD Visual Odometry
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            name='disparity_node',
            parameters=[disparity_params],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ],
            condition=IfCondition(LaunchConfiguration('rgbd_mode')),
        ),

        ComposableNode(
            package='stereo_cam',
            plugin='stereo_cam::DepthNode',
            name='depth_image_node',
            extra_arguments=[
                {'use_intra_process_comms': True},
            ],
            condition=IfCondition(LaunchConfiguration('rgbd_mode')),
        ),

        ComposableNode(
            package='rtabmap_odom',
            plugin='rtabmap_odom::RGBDOdometry',
            name='rtabmap_odom',
            parameters=[rtab_rgbd_params],
            remappings=[
                ('rgb/image', '/left/image_rect'),
                ('rgb/camera_info', '/left/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
            ],
            extra_arguments=[
                {'use_intra_process_comms': True},
            ],
            condition=IfCondition(LaunchConfiguration('rgbd_mode'))
        ),

    ]

    # container name (auto unless user overrides)
    ctn_name = PythonExpression([
        '"', LaunchConfiguration('namespace'), '/stereo_container"'
        ' if "', LaunchConfiguration('namespace'), '" != "" '
        'else "stereo_container"'
    ])

    container = ComposableNodeContainer(
        name=ctn_name,
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_intra_process_comms': True}],
        composable_node_descriptions=components,
        output='screen', 
        arguments=['--ros-args', '--log-level', log_level]
    )
    return [container]

def generate_launch_description():

    return LaunchDescription([

        # CLI args (same semantics as your legacy file)
        DeclareLaunchArgument('use_raspi',  default_value='false'),
        DeclareLaunchArgument('enable_disparity', default_value='false'),
        DeclareLaunchArgument('rgbd_mode', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='WARN'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace',  default_value=''),

        # one container with everything
        OpaqueFunction(function=compose_perception),

        # robot_state_publisher outside (doesn’t need zero-copy)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description':
                    Command(['xacro ', os.path.join(
                        get_package_share_directory('stereo_cam'),
                        'urdf', 'stereo_camera.urdf.xacro'
                    )]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])
