from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unicore_gnss',
            executable='um982_node',
            name='um982_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('unicore_gnss'),
                    'config',
                    'um982.yaml',
                ])
            ],
        )
    ])
