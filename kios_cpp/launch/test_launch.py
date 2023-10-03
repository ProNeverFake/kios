from launch.actions import OpaqueFunction

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def launch_in_new_terminal(cmd, log_file):
    """
    Helper function to spawn a new terminal and run a command.
    Adjust for your specific terminal if not using gnome-terminal.
    """
    def fn(context):
        subprocess.Popen(['gnome-terminal', '--', 'bash',
                         '-c', f'{cmd} | tee {log_file}'])
        return []

    return OpaqueFunction(function=fn)

def generate_launch_description():

    mongo_reader = Node(
        package='kios_cpp',
        namespace='',
        executable='mongo_reader',
        name='mongo_reader'
    )

    mios_reader = Node(
        package='kios_py',
        namespace='',
        executable='mios_reader',
        name='mios_reader'
    )

    messenger = Node(
        package='kios_cpp',
        namespace='',
        executable='messenger',
        name='messenger'
    )

    tree_node = Node(
        package='kios_cpp',
        namespace='',
        executable='tree_node',
        name='tree_node'
    )

    tactician = Node(
        package='kios_cpp',
        namespace='',
        executable='tactician',
        name='tactician'
    )

    commander = Node(
        package='kios_cpp',
        namespace='',
        executable='commander',
        name='commander'
    )

    return LaunchDescription([
        # background_r_launch_arg,
        # background_g_launch_arg,
        # background_b_launch_arg,
        # chatter_ns_launch_arg,
        # launch_include,
        # launch_include_with_namespace,
        # turtlesim_node,
        # turtlesim_node_with_parameters,
        # forward_turtlesim_commands_to_second_turtlesim_node,
        mios_reader,
        mongo_reader,
        messenger,
        # tree_node,
        tactician,
        launch_in_new_terminal('ros2 run kios_cpp commander', 'commander.log'),
        # launch_in_new_terminal('ros2 run kios_cpp mongo_reader', 'mongo_reader.log'),
        # launch_in_new_terminal('ros2 run kios_py mios_reader', 'mios_reader.log'),
        # launch_in_new_terminal('ros2 run kios_cpp messenger', 'messenger.log'),
        # launch_in_new_terminal('ros2 run kios_cpp tactician', 'tactician.log'),
        launch_in_new_terminal('ros2 run kios_cpp tree_node', 'tree_node.log'),
    ])

