
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_path = get_package_share_path('camera')
    default_rviz_config_path  = camera_path / 'april.rviz'
    default_april_config_path = camera_path / 'april.yaml'

    rviz_launch_arg = DeclareLaunchArgument(
        name='rviz_pub', 
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable rviz2')

    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    cv_node = Node(
        package='camera',
        executable='cam',
        output='screen'
    )

    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch/rs_launch.py'
                ])
            ]),
        launch_arguments=[('depth_module.profile', '1280x720x30'),
                          ('pointcloud.enable', 'true'),
                          ('align_depth.enable', 'true')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=LaunchConfigurationEquals('rviz_pub', 'true')
    )

    april_node = Node(
        package ='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        remappings=[('/image_rect', '/camera/color/image_raw'),
                   ('/camera_info', '/camera/color/camera_info')],
        parameters=[default_april_config_path]
    )

    return LaunchDescription([
        launch_realsense,
        cv_node,
        rviz_config_arg,
        rviz_launch_arg,
        rviz_node,
        april_node
    ])