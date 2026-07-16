"""Launch the standalone central-PC presentation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    start_web = LaunchConfiguration('start_web')
    allow_degraded_drive = LaunchConfiguration('allow_degraded_drive')
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_web',
            default_value='true',
            description='Start the built-in web UI and REST API',
        ),
        DeclareLaunchArgument(
            'allow_degraded_drive',
            default_value='false',
            description=(
                'Demo-only: let straight routes and workflows ignore stale '
                'odometry and central velocity graph readiness'
            ),
        ),
        Node(
            package='final_project_presentation2',
            executable='final_project_presentation2',
            name='final_project_presentation2',
            output='screen',
            parameters=[{
                'start_web': ParameterValue(start_web, value_type=bool),
                'allow_degraded_drive': ParameterValue(
                    allow_degraded_drive,
                    value_type=bool,
                ),
            }],
        ),
    ])
