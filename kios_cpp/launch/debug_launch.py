import launch
from launch import LaunchDescription
from launch.actions import OpaqueFunction

import subprocess


def launch_in_new_terminal(cmd):
    """
    Helper function to spawn a new terminal and run a command.
    Adjust for your specific terminal if not using gnome-terminal.
    """

    def fn(context):
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", cmd])
        return []

    return OpaqueFunction(function=fn)


def generate_launch_description():
    return LaunchDescription(
        [
            launch_in_new_terminal("ros2 run kios_cpp commander"),
            launch_in_new_terminal("ros2 run kios_cpp mongo_reader"),
            launch_in_new_terminal("ros2 run kios_py mios_reader"),
            launch_in_new_terminal("ros2 run kios_cpp messenger"),
            launch_in_new_terminal("ros2 run kios_cpp tactician"),
            launch_in_new_terminal("ros2 run kios_cpp tree_node"),
        ]
    )
