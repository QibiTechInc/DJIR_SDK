from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    can_interface = LaunchConfiguration('can_interface')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('djir_ros2_driver'), 'config', 'djir_driver.yaml'])),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        Node(
            package='djir_ros2_driver',
            executable='djir_driver_node',
            name='djir_driver',
            output='screen',
            parameters=[config, {'can_interface': can_interface}],
        ),
    ])
