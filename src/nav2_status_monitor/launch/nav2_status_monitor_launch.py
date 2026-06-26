# Nav2 status monitor: subscribe to Nav2 action status/feedback and behavior tree logs;
# publish simplified Nav2Status messages on /nav2/status.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_status_monitor')
    default_config = os.path.join(pkg_share, 'config', 'nav2_status_monitor.yaml')

    config_file = LaunchConfiguration('config_file', default=default_config)
    status_topic = LaunchConfiguration('status_topic', default='/nav2/status')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='2.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Full path to nav2_status_monitor parameter file.',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value=status_topic,
            description='Output nav2_status_monitor/Nav2Status topic.',
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='Periodic status summary interval in seconds (0 to disable).',
        ),

        Node(
            package='nav2_status_monitor',
            executable='nav2_status_monitor_node',
            name='nav2_status_monitor',
            output='screen',
            parameters=[
                config_file,
                {
                    'status_topic': status_topic,
                    'publish_period_sec': publish_period_sec,
                },
            ],
        ),
    ])
