from launch import LaunchDescription
from launch_ros.actions import Node
from launch_helpers import get_ws_src_directory, add_namespace_to_yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import os

namespace_ = 'marta'

# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('gamepad_parser')

def generate_launch_description():
    # Individual Parameter files
    gamepad_parser_config = os.path.join(ros2_ws_src, 'gamepad_parser', 'config', 'gamepad_parser.yaml')

    # Add namespace to the yaml file
    gamepad_parser_config_ns = add_namespace_to_yaml(namespace_, gamepad_parser_config)

    # Print Config File
    # with open(gamepad_parser_config_ns, 'r') as f:
    #     print(f.read())

    return LaunchDescription([
        Node(
            package='gamepad_parser',
            node_namespace=namespace_,
            node_executable='gamepad_parser_node',
            node_name='gamepad_parser_node',
            output='screen',
            parameters=[gamepad_parser_config_ns],
            emulate_tty=True
        )
    ])
