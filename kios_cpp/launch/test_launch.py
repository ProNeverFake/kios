from launch import LaunchDescription
from launch.actions import OpaqueFunction

import subprocess


# def launch_in_new_terminal(cmd, log_file):
#     """
#     Helper function to spawn a new terminal and run a command.
#     Adjust for your specific terminal if not using gnome-terminal.
#     """
#     def fn(context):
#         subprocess.Popen(['gnome-terminal', '--', 'bash',
#                          '-c'])
#         return []

#     return OpaqueFunction(function=fn)

# def launch_in_new_terminal(cmd, log_file):
#     """
#     Helper function to spawn a new terminal and run a command.
#     Adjust for your specific terminal if not using gnome-terminal.
#     """
#     def fn(context):
#         subprocess.Popen(['gnome-terminal', '--', 'bash',
#                          '-c', f'{cmd} > {log_file} 2>&1'])
#         return []

#     return OpaqueFunction(function=fn)


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

# def launch_in_new_terminal(cmd, log_file):
#     """
#     Helper function to spawn a new terminal and run a command.
#     Adjust for your specific terminal if not using gnome-terminal.
#     """
#     def fn(context):
#         cmd_with_log_env = f"export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1; {cmd} | tee {log_file}"
#         subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd_with_log_env])
#         return []

#     return OpaqueFunction(function=fn)

def generate_launch_description():
    return LaunchDescription([
        launch_in_new_terminal('ros2 run kios_cpp commander', 'commander.log'),
        launch_in_new_terminal('ros2 run kios_py mongo_reader', 'mongo_reader.log'),
        launch_in_new_terminal('ros2 run kios_py mios_reader', 'mios_reader.log'),
        launch_in_new_terminal('ros2 run kios_cpp messenger', 'messenger.log'),
        launch_in_new_terminal('ros2 run kios_cpp tactician', 'tactician.log'),
        launch_in_new_terminal('ros2 run kios_cpp tree_node', 'tree_node.log'),
    ])
