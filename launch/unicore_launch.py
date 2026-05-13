from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_unicore_node(
    *,
    executable: str = "unicore_node",
    node_name: str = "unicore_node",
    config_name: str = "unicore.yaml",
):
    return Node(
        package="unicore_gnss",
        executable=executable,
        name=node_name,
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("unicore_gnss"),
                    "config",
                    config_name,
                ]
            )
        ],
    )


def generate_launch_description():
    return LaunchDescription([make_unicore_node()])
