import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PythonExpression, Command
)
from launch_ros.actions import (
    ComposableNodeContainer, LoadComposableNodes, Node
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

    rtab_params = ParameterFile(os.path.join(
        pkg_dir, 'config/odom/rtab_odom.yaml'
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
                {'enable_depth':  LaunchConfiguration('enable_depth')},
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
            ]
        ),
        # 3 disparity
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            name='disparity_node'
        ),
        # 4 stereo VO
        ComposableNode(
            package='rtabmap_odom',
            plugin='rtabmap_odom::StereoOdometry',
            name='rtabmap_odom',
            parameters=[rtab_params]
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
        output='screen'
    )
    return [container]

# ───────────────────────────── launch description ──────────────────────────
def generate_launch_description():

    return LaunchDescription([

        # CLI args (same semantics as your legacy file)
        DeclareLaunchArgument('use_raspi',  default_value='false'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('rviz',       default_value='false'),
        DeclareLaunchArgument('namespace',  default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

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
            output='screen'
        ),

        # optional RViz
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('stereo_cam'),
                'config', 'stereo_cam.rviz'
            )],
            output='screen'
        ),
    ])
