"""
Launch the GUI to simulate the LfD framework with the AGX Simulator.

Use with: python lfd/lfd_gui_mp.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05
from the /algoryx folder
"""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from copy import copy, deepcopy
from distutils.dir_util import copy_tree
import glob
import logging
import multiprocessing as mp
import os
import subprocess
import time
from typing import Any, List

from agxBrick.brickLoaderUtils import createArgumentParser
from bt_learning.learning_from_demo.clustering import find_equivalent_actions
from bt_learning.learning_from_demo.learning import learn_tree
from bt_learning.learning_from_demo.plot_clusters import plot_clusters
import bt_learning.learning_from_demo.render_tree as tree_render
from py_trees.display import render_dot_tree
import robot_behaviors.mobile_base_behaviors.lfd_actions as base_actions
import robot_behaviors.yumi_behaviors.lfd_actions as yumi_actions
from robot_interface.demonstration import RobotDemonstration
import simulation.algoryx.agx_application as app
from simulation.algoryx.behaviors.sim_behaviors import RobotBehaviors
from simulation.algoryx.lfd import lfd_asprocess, lfd_interface
from simulation.algoryx.lfd.planning_itnerface import PlanningInterface
from simulation.py_trees_interface import PyTree, PyTreeParameters
from user_interface.gui_ddmenu import GUIMenu
from user_interface.lfd_gui import LfDGUILayout
from user_interface.lfd_gui_actions import GUIActions
import yaml


# OS dependent import
if os.name == 'nt':  # Windows
    import PySimpleGUIQt as gui
elif os.name == 'posix':  # Linux Ubuntu
    import PySimpleGUI as gui


class LFD_GUI():

    def __init__(self, args: Any):
        self._logger = logging.getLogger(__file__)

        self.repo_path = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
        with open(os.path.join(self.repo_path, 'config/sim_data.yaml')) as f:
            sim_data = yaml.safe_load(f)
        with open(os.path.join(self.repo_path, 'config/sim_objects.yaml')) as f:
            self.obj_data = yaml.safe_load(f)

        self.objects = list(self.obj_data.keys())

        self.default_frame = sim_data['demonstration']['default_frame']
        self.robot_frame = sim_data['demonstration']['robot_frame']
        self.ee_frame = sim_data['algoryx']['ee_name']

        self.agx_GUI = LfDGUILayout('Algoryx Simulation for Learning from Demonstration')
        self.all_frames = sim_data['demonstration']['reference_frames'] + self.objects
        if self.robot_frame not in self.all_frames:
            self.all_frames.append(self.robot_frame)
        if self.default_frame not in self.all_frames:
            self.all_frames.append(self.default_frame)
        self.pick_menu = GUIMenu('Choose the object to pick.', self.objects)

        self.grasping_actions = sim_data['demonstration']['grasping_actions']
        self.placing_actions = sim_data['demonstration']['placing_actions']
        self.navigation_actions = sim_data['demonstration']['navigation_actions']

        self.actions = self.grasping_actions + self.placing_actions

        # demo flags
        self.demo_dir = None
        self.demonstrations = None

        # bt flags
        self.running = False
        self.bt_length = 0
        self.bt_tick_freq = sim_data['behavior_tree']['tick_freq']

        # action flags
        self.holding = None
        self.running_action = None

        self.args = args

        self.interface = lfd_interface.LfDInterface(self.objects)
        self.parent_conn, self.child_conn = mp.Pipe()

    def show(self, build_tree_function: Any = None):
        """Define the GUI functionalities."""
        self.agx_GUI.initialize_layout()

        window = self.agx_GUI.get_window()

        # Interaction with the GUI Main menu
        ret = 0
        while ret is not None:
            ret, values = window.read()

            if ret == '__folder_display__' and os.path.isdir(values['__folder_display__']):
                self.demo_dir = values['__folder_display__']
                self.__reload_demos(window, build_tree_function)

            elif ret == '__new_demo__':
                window.disappear()
                self.__new_demo()
                self.__reload_demos(window, build_tree_function)
                window.reappear()

            elif ret == '__run__':
                if self.running is False:
                    # disable all elements
                    window.find_element('__show_tree__').update(disabled=True)
                    window.find_element('__run__').update('Stop')
                    print('Launching tree...')
                    self.running = True

                    behaviors = RobotBehaviors(os.path.join(self.tree_dir, 'settings'))
                    self.bt_process = app.RunBT(
                        self.interface,
                        self.obj_data,
                        behaviors,
                        self.tree_dir,
                        self.bt_tick_freq,
                        (app.CloudpickleWrapper(app.Application), self.args)
                    )
                    self.bt_process.start()

                else:
                    self.running = False
                    window.find_element('__run__').update('Run')
                    self.bt_process.stop()
                    self.bt_process.join()
                    window.find_element('__show_tree__').update(disabled=False)

            elif ret == '__show_tree__':
                if os.name == 'nt':  # Windows
                    os.startfile(os.path.join(self.tree_dir, 'full.svg'))
                else:
                    opener = 'xdg-open'
                    subprocess.call([opener, os.path.join(self.tree_dir, 'full.svg')])

            elif ret == '__save_tree__':
                folder = gui.popup_get_folder('Select a folder')
                if folder is not None and folder != '':
                    if os.path.isdir(folder):
                        number = 1
                        while os.path.isdir(os.path.join(folder, f'bt{number}')):
                            number += 1

                        folder = os.path.join(folder, f'bt{number}')

                os.makedirs(folder)
                copy_tree(self.tree_dir, folder)
                print('Tree saved!')

            elif ret == '__load_tree__':
                folder = gui.popup_get_folder('Select a folder')
                if folder is not None and folder != '':
                    if os.path.isdir(folder):
                        self.tree_dir = folder

                if build_tree_function is None:
                    self.__build_tree(window, from_file=True)
                else:
                    build_tree_function(window, from_file=True)

            elif ret == '__plot_cluster__':
                if self.demonstrations is None:
                    _ = self.__load_demo()
                plot_clusters(self.demo_dir, self.demonstrations)

    def __new_demo(self):
        """Launch a new window to start a new demonstration."""
        action_GUI = GUIActions('New demo', self.actions)
        action_GUI.initialize_layout()

        action_window = action_GUI.get_window()

        if self.demonstrations is None:
            n_demos = 0
        else:
            n_demos = self.demonstrations.n_demonstrations()

        os.mkdir(os.path.join(self.demo_dir, 'demo%d' % (n_demos + 1)))

        config_path = os.path.join(self.repo_path, 'config')
        self.lfd_process = lfd_asprocess.LfDProcess(
            config_path,
            self.interface,
            (self.child_conn, app.CloudpickleWrapper(app.Application), self.args)
        )
        self.lfd_process.start()

        ret = 0
        while ret is not None:
            ret, values = action_window.read()
            action_GUI.execute_action(ret, self.__manipulate, n_demos)

        self.lfd_process.stop()
        self.lfd_process.join()

    def __manipulate(self, n_demos: int, action_name: str, gripper_state: str):
        """Define how to run actions when connected to the robot."""
        def get_obj_from_menu(menu: GUIMenu) -> str:
            menu_ = deepcopy(menu)
            key_ = menu_.initialize_layout()
            window = menu_.get_window()
            event, values = window.read()
            obj = values[key_]
            window.close()
            return obj

        # Interact with the GUI Action menu
        if action_name == 'pick':
            # Handle dropdown menu
            self.holding = get_obj_from_menu(self.pick_menu)
            self.parent_conn.send(('pick', self.holding))
            print(f'Picking {self.parent_conn.recv()}.')
            self.__write_action(action_name, n_demos + 1, self.parent_conn)
        else:
            mainpulating = copy(self.holding)
            print(f'Releasing {mainpulating}.')
            self.parent_conn.send((action_name, ()))
            print(f'Holding {self.parent_conn.recv()}.')
            self.__write_action(action_name, n_demos + 1, self.parent_conn)
            self.parent_conn.send(('lift', ()))

    def __write_action(self, action_type: str, current_demo: str, conn: mp.connection.Connection):
        """Write the pose of the end effector in all available frames as a new action."""
        action_data = {'type': action_type, 'vec_pos': {}, 'vec_quat': {}}
        current_demo_dir = os.path.join(self.demo_dir, 'demo%d' % current_demo)
        if self.demonstrations is None:
            frames = self.all_frames
        else:
            frames = self.demonstrations.frames

        for frame in frames:
            # remove the default frame for now
            conn.send(('get_sensor_data', (self.ee_frame, frame)))
            position, orientation = conn.recv()

            action_data['vec_pos'][frame] = position.tolist()
            action_data['vec_quat'][frame] = orientation.tolist()

        action_number = len(glob.glob(os.path.join(current_demo_dir, 'data_*.yaml'))) + 1
        with open(os.path.join(current_demo_dir, f'data_{action_number}.yaml'), 'w') as f:
            yaml.dump(action_data, f, default_flow_style=None)

    def __reload_demos(self, window: gui.Window, build_tree_function):
        """Update the demonstration history and build the tree."""
        n_demos = self.__load_demo()
        window.find_element('__n_demos__').update('Number of demonstrations: %d' % n_demos)
        window.find_element('__new_demo__').update(disabled=False)

        if self.demonstrations is None:
            return

        if build_tree_function is None:
            self.__build_tree(window)
        else:
            build_tree_function(window)

    def __load_demo(self) -> int:
        """Load demonstration and return number of demonstrations."""
        info_file = os.path.join(self.demo_dir, 'info.yaml')
        if not os.path.isfile(info_file):
            with open(info_file, 'w') as f:
                yaml.dump(
                    {
                        'frames': self.all_frames,
                        'robot': self.robot_frame,
                        'default_frame': self.default_frame
                    }, f)
            self.demonstrations = None
            return 0
        elif len(glob.glob(os.path.join(self.demo_dir, 'demo*'))) == 0:
            self.demonstrations = None
            return 0
        else:
            # Remove empty folders
            for folder in glob.glob(self.demo_dir + '/demo[0-9]*/'):
                # Each demonstration has to contain at least one action
                if not os.path.isfile(folder + '/data_1.yaml'):
                    print(f'Folder {folder} missing data file, removing.')
                    os.rmdir(folder)
            self.demonstrations = RobotDemonstration(
                self.demo_dir,
                custom_actions={
                    'pick': yumi_actions.PickAction,
                    'place': yumi_actions.PlaceAction,
                    'drop': yumi_actions.PlaceAction,
                    'move': base_actions.MoveAction
                },
                exclude_frames={
                    'pick': [self.robot_frame, self.default_frame],
                    'place': [],
                    'drop': [],
                    'move': [self.robot_frame]
                }
            )
            return self.demonstrations.n_demonstrations()

    def __bt_worker(
        self,
        conn: mp.connection.Connection,
        target_directory: str,
        string_tree: str,
        behaviors: Any,
        env_fn_wrapper: app.CloudpickleWrapper,
        args: List[str]
    ):
        app = env_fn_wrapper.var(args)
        app.bringup(self.obj_data, visual=False)
        self.interface.initialize()
        parameters = PyTreeParameters()
        parameters.behavior_lists = behaviors.get_behavior_list()
        parameters.behaviors = behaviors
        bt = PyTree(
            string_tree, parameters, world_interface=self.interface)

        render_dot_tree(
            bt.root,
            name='debug_pytree',
            target_directory=target_directory,
            with_blackboard_variables=True
        )

        if conn.poll():
            data = conn.recv()
            conn.send(bt.bt.length())

        tree_render.dot_graph(bt).write_svg(
            os.path.join(target_directory, 'full.svg'), encoding='utf-8')
        positions = tree_render.write_tikz_tree(
            bt, os.path.join(target_directory, 'full.tex'))
        tree_render.py_trees_dot(bt).write_svg(
            os.path.join(target_directory, 'full_pytrees.svg'), encoding='utf-8')

        with open(os.path.join(target_directory, 'positions.yaml'), 'w') as f:
            yaml.dump(positions, f)

        print('BT data saved!')

        app.shutdown()

    def __save_tree(
        self,
        string_tree: str,
        target_directory: str,
        behaviors: Any
    ):
        """Prepare the BT data for visualization and execution."""
        args = (
            self.child_conn,
            target_directory,
            string_tree,
            behaviors,
            app.CloudpickleWrapper(app.Application),
            self.args
        )
        proc = mp.Process(target=self.__bt_worker, args=args)
        proc.start()
        time.sleep(2)
        self.parent_conn.send('')
        self.bt_length = self.parent_conn.recv()
        time.sleep(2)
        proc.join()
        print('Behavior Tree generated!')

    def __prelearning_handler(self, window: gui.Window) -> gui.ProgressBar:
        """Update window behavior before learning the tree."""
        window.find_element('__run__').update('Building tree...', disabled=True)
        window.find_element('__show_tree__').update(disabled=True)
        window.find_element('__save_tree__').update(disabled=True)
        window.find_element('__load_tree__').update(disabled=True)
        window.find_element('__plot_cluster__').update(disabled=True)
        pbar = window.find_element('__progress__')
        pbar.update(visible=True)
        window.read(timeout=0)

        return pbar

    def __postlearning_handler(self, window: gui.Window, pbar: gui.ProgressBar):
        """Update window behavior before learning the tree."""
        window.find_element('__run__').update('Run', disabled=False)
        window.find_element('__show_tree__').update(disabled=False)
        window.find_element('__save_tree__').update(disabled=False)
        window.find_element('__load_tree__').update(disabled=False)
        window.find_element('__plot_cluster__').update(disabled=False)
        window.find_element('__n_nodes__').update('Number of nodes: %d' % self.bt_length)
        pbar.update(visible=False)

    def __get_tree_from_file(self):
        """Load an existing BT from a file."""
        # Use a learnt BT
        bt_description = os.path.join(self.tree_dir, 'tree.yaml')
        try:
            with open(bt_description) as f:
                string_tree = yaml.safe_load(f)
            behaviors = RobotBehaviors(os.path.join(self.tree_dir, 'settings'))
            self.__save_tree(string_tree, self.tree_dir, behaviors)
        except FileNotFoundError:
            print('Error in loading BT description.')

    def __learn_tree(self, pbar: gui.ProgressBar) -> str:
        """
        Prepare the Behavior Tree and return its string representation.

        The argument post_processing is a function that modifies the learnt BT.
        """
        # Learn the BT
        self.tree_dir = os.path.join(self.repo_path, 'lfd/demos_mp/tmp_dir')
        if not os.path.isdir(self.tree_dir):
            os.mkdir(self.tree_dir)
        settings_dir = os.path.join(self.tree_dir, 'settings')

        behaviors = RobotBehaviors(settings_dir)
        # Send the offline interface to the planner to expand the BT
        offline_interface = PlanningInterface(
            available_objects=self.objects,
            frames=self.demonstrations.frames,
            default_frame=self.demonstrations.default_frame,
            random_events=False,
            robot_frame=self.robot_frame
        )

        equivalent = find_equivalent_actions(
            self.demonstrations,
            {
                'pick': yumi_actions.EquivalentPick,
                'place': yumi_actions.EquivalentPlace,
                'drop': yumi_actions.EquivalentPlace,
                'move': base_actions.EquivalentMove
            }
        )

        # tree is a PyTree object!
        tree = learn_tree(
            settings_dir,
            equivalent,
            behaviors,
            offline_interface,
            iterations=50,
            callback=pbar.update_bar
        )

        string_tree = tree.bt.bt

        # in string_tree, the string representation of the tree is stored
        with open(os.path.join(self.tree_dir, 'tree.yaml'), 'w') as f:
            yaml.dump(string_tree, f)

        self.__save_tree(string_tree, self.tree_dir, behaviors)

    def __build_tree(
        self,
        window: gui.Window,
        from_file: bool = False,
    ):
        """
        Build the Behavior Tree once it is loaded.

        The argument post_processing is a function that modifies the learnt BT.
        """
        pbar = self.__prelearning_handler(window)

        if from_file:
            self.__get_tree_from_file()
        else:
            self.__learn_tree(pbar)

        self.__postlearning_handler(window, pbar)


def main():
    parser, args, leftover_args = createArgumentParser()

    gui = LFD_GUI(args)
    gui.show()


if __name__ == '__main__':
    main()
