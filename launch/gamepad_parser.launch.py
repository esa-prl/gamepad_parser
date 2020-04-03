from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

import xacro
import tempfile

namespace_ = 'marta'

ros2_ws_src = '/home/freki/rover_wss/ros2_ws/src'

def generate_launch_description():
    ## Individual Parameter files
    # Loading it from the install space
    # gamepad_parser_dir = get_package_share_directory('gamepad_parser')
    # gamepad_parser_config = os.path.join(gamepad_parser_dir, 'gamepad_parser.yaml')

    gamepad_parser_dir = os.path.join(ros2_ws_src, 'gamepad_parser')
    gamepad_parser_config = os.path.join(gamepad_parser_dir, 'config', 'gamepad_parser.yaml')

    ## Add namespace to the yaml file
    gamepad_parser_config_ns = add_namespace_to_yaml(namespace_, gamepad_parser_config)

    with open(gamepad_parser_config_ns, 'r') as f:
        print(f.read())

    return LaunchDescription([
        Node(
            package='gamepad_parser',
            node_namespace=namespace_,
            node_executable='gamepad_parser_node',
            node_name='gamepad_parser_node',
            output='screen',
            parameters=[gamepad_parser_config_ns],
            # arguments=[gamepad_parser_config],
            # arguments=['--ros-args --params-file /home/freki/rover_wss/ros2_ws/src/gamepad_parser/config/gamepad_parser.yaml'],
            emulate_tty=True
        )
    ])

def add_namespace_to_yaml(namespace, yaml_path, ns_yaml_path=None):
    """Make config files reusable in multiple namespaces by generating a new yaml with a namespace appended
    * yaml_path -- the path to the yaml file
    * ns_yaml_path -- the path to the namespaced yaml file
    """
    # If no YAML path is given, use a temporary file
    if ns_yaml_path is None:
        ns_yaml_path = tempfile.mktemp(prefix="%s_" %
                                    os.path.basename(yaml_path))

    # open and process file
    with open(yaml_path, 'r') as yaml_file, open(ns_yaml_path, 'w') as ns_yaml_file:
        ns_yaml_file.write('# Temporary Namespace\n')     
        ns_yaml_file.write(namespace + ':\n')     
        for line in yaml_file:
            ns_yaml_file.write("  " + line)
    return ns_yaml_path  # Return path to the urdf file

