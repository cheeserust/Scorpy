"""Launch only the Pinky-side raw velocity watchdog."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    allow_unsynchronized_clock = LaunchConfiguration(
        'allow_unsynchronized_clock'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'allow_unsynchronized_clock',
            default_value='false',
            description=(
                'Demo-only fallback to local receipt time when the central '
                'PC and Pinky clocks are not synchronized'
            ),
        ),
        Node(
            package='final_project_presentation2',
            executable='final_project_presentation2_watchdog',
            name='final_project_presentation2_watchdog',
            output='screen',
            parameters=[{
                'allow_unsynchronized_clock': ParameterValue(
                    allow_unsynchronized_clock,
                    value_type=bool,
                ),
            }],
            respawn=True,
            respawn_delay=0.25,
        ),
    ])
