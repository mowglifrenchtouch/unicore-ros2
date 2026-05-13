from launch import LaunchDescription

from unicore_launch import make_unicore_node


def generate_launch_description():
    return LaunchDescription(
        [
            make_unicore_node(
                executable="um982_node",
                node_name="um982_node",
                config_name="um982.yaml",
            )
        ]
    )
