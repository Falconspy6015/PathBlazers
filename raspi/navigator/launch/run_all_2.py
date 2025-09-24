# slam_ekf_launch.py
#
# Launch file to start:
#   1. slam_toolbox in online_async mode
#   2. robot_localization's ekf_node with a given configuration file
#
# Usage:
#   ros2 launch your_package slam_ekf_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare arguments for config files
    slam_params_file = LaunchConfiguration('slam_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            os.getenv('HOME'),
            '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml'
        ),
        description='Full path to the slam_toolbox parameters file.'
    )

    declare_ekf_params_cmd = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(
            os.getenv('HOME'),
            '/opt/ros/jazzy/share/robot_localization/params/ekf.yaml'
        ),
        description='Full path to the ekf_node parameters file.'
    )

    # Start slam_toolbox (online_async mode)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    # Start EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            # Example: remap odom topic
            ('/odometry/filtered', '/odom')
        ]
    )
    
    # Static transform: base_link -> laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        # args: x y z roll pitch yaw parent child
    )

    # Static transform: base_link -> base_footprint
    static_tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription([
        declare_slam_params_cmd,
        declare_ekf_params_cmd,
        slam_toolbox_node,
        ekf_node,
        static_tf_base_to_laser,
        static_tf_base_to_footprint
    ])

