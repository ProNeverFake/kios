"""
Launch the GUI to simulate the GP+LfD framework with the AGX Simulator.

Use with: python combined/gui.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05
from the /algoryx folder
"""
from copy import copy, deepcopy
import glob
import logging
import multiprocessing as mp
import os
import re
import subprocess
import time
from typing import Any, Dict, List

from agxBrick.brickLoaderUtils import createArgumentParser
from .behaviors.behavior_lists import BehaviorLists
from bt_learning.gp import logplot
from bt_learning.learning_from_demo.clustering import find_equivalent_actions
from bt_learning.learning_from_demo.learning import learn_tree
from bt_learning.learning_from_demo.plot_clusters import plot_clusters
import robot_behaviors.mobile_base_behaviors.lfd_actions as base_actions
import robot_behaviors.yumi_behaviors.lfd_actions as yumi_actions
from robot_interface.demonstration import RobotDemonstration
import simulation.algoryx.agx_application as app
import simulation.algoryx.combined.gui_common as func
import simulation.algoryx.combined.processes as proc
from simulation.algoryx.behaviors.sim_behaviors import RobotBehaviors
from simulation.algoryx.gp import gp_asprocess, gp_environment, gp_interface
from simulation.algoryx.lfd import lfd_asprocess, lfd_interface
from simulation.algoryx.lfd.planning_itnerface import PlanningInterface
from user_interface.agx_gui import AGXGUILayout
from user_interface.gui_ddmenu import GUIMenu
from user_interface.lfd_gui_actions import GUIActions
import yaml

# OS dependent import
if os.name == "nt":  # Windows
    import PySimpleGUIQt as gui
elif os.name == "posix":  # Linux Ubuntu
    import PySimpleGUI as gui


class GUI:
    def __init__(self, args: Any) -> None:
        self._logger = logging.getLogger(__file__)

        self.repo_path = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
        with open(os.path.join(self.repo_path, "config/sim_data.yaml")) as f:
            self.sim_data = yaml.safe_load(f)
        with open(os.path.join(self.repo_path, "config/sim_objects.yaml")) as f:
            self.obj_data = yaml.safe_load(f)
        with open(os.path.join(self.repo_path, "config/gp_targets.yaml")) as f:
            self.config_targets = yaml.safe_load(f)
        with open(os.path.join(self.repo_path, "config/gp_initial.yaml")) as f:
            gp_initial = yaml.safe_load(f)

        self.targets = copy(self.config_targets)
        self.task_objects = list(self.obj_data.keys())
        self.ref_objects = self.sim_data["demonstration"]["reference_frames"]

        # AGX Stuff
        self.agx_args = args
        random_bringup = self.sim_data["algoryx"]["random_bringup"]
        self.initial = gp_initial if random_bringup is False else None

        # BT Stuff
        self.bt = None
        self.bt_length = 0
        self.bt_tick_freq = self.sim_data["behavior_tree"]["tick_freq"]
        self.behaviors = None

        # GP Stuff
        app.GRAPHICS = False
        self.gp_interface = gp_interface.GPInterface(
            self.task_objects, self.ref_objects
        )
        self.gp_par = func.init_gp_params(self.sim_data, self.repo_path)
        self.baseline_index = self.sim_data["genetic"]["baseline_index"]
        if type(self.baseline_index) is not int or self.baseline_index == "None":
            self.baseline_index = None
        self.fitness_coeff = func.get_fitness_coefficients()
        self.visual = self.sim_data["genetic"]["graphics"]

        self.log_folder = os.path.join(
            self.repo_path, f"logs/log_{self.gp_par.log_name}"
        )
        self.gp_process = None
        self.current_gen = 0

        # LfD Stuff
        app.GRAPHICS = False
        self.lfd_interface = lfd_interface.LfDInterface(self.task_objects)
        self.default_frame = self.sim_data["demonstration"]["default_frame"]
        self.robot_frame = self.sim_data["demonstration"]["robot_frame"]
        self.ee_frame = self.sim_data["algoryx"]["ee_name"]

        self.all_frames = (
            self.sim_data["demonstration"]["reference_frames"] + self.task_objects
        )
        if self.robot_frame not in self.all_frames:
            self.all_frames.append(self.robot_frame)
        if self.default_frame not in self.all_frames:
            self.all_frames.append(self.default_frame)
        self.pick_menu = GUIMenu("Choose the object to pick.", self.task_objects)

        self.grasping_actions = self.sim_data["demonstration"]["grasping_actions"]
        self.placing_actions = self.sim_data["demonstration"]["placing_actions"]
        self.navigation_actions = self.sim_data["demonstration"]["navigation_actions"]

        self.actions = self.grasping_actions + self.placing_actions

        self.demo_folder = os.path.join(self.log_folder, "demos")
        self.lfd_tree_folder = os.path.join(self.demo_folder, "bt1")
        self.demonstrations = None

        # Action flags
        self.holding = None
        self.running_action = None

        self.parent_conn, self.child_conn = mp.Pipe()

        # Initialize GUI
        self.agx_GUI = AGXGUILayout(
            "Simulation framework for learning Behavior Trees",
            os.path.join(self.repo_path, f"logs"),
        )

    def show(self, build_tree_function: Any = None) -> None:
        """Define the GUI functionalities."""
        self.agx_GUI.initialize_layout()

        window = self.agx_GUI.get_window()

        # flags:
        has_BT = False
        has_GP = False
        has_Demo = False

        env_showing = False
        gp_running = False
        bt_running = False

        # Interaction with the GUI Main menu
        ret = 0
        while ret is not None:
            ret, values = window.read()

            if ret == "__folder_display__" and os.path.isdir(
                values["__folder_display__"]
            ):
                self.log_folder = values["__folder_display__"]
                self.demo_folder = os.path.join(self.log_folder, "demos")
                self.lfd_tree_folder = os.path.join(self.demo_folder, "bt1")
                if not os.path.isdir(self.demo_folder):
                    os.makedirs(self.lfd_tree_folder)
                try:
                    id = int(self.log_folder[-2:])
                    self.gp_par.log_name = str(id)
                except ValueError:
                    self.gp_par.log_name = self.log_folder[-1]
                self.gp_par.behavior_lists = self._generate_behaviors(self.log_folder)
                self.behaviors = RobotBehaviors(self.log_folder, self.task_objects)
                try:
                    with open(os.path.join(self.log_folder, "tree.yaml"), "r") as f:
                        self.bt = yaml.safe_load(f)
                except FileNotFoundError:
                    pass
                try:
                    with open(
                        os.path.join(self.log_folder, "gp_targets.yaml"), "r"
                    ) as f:
                        self.targets = yaml.safe_load(f)
                except FileNotFoundError:
                    pass
                self._update_gp_values(window)
                self._reload_demos(window, build_tree_function)
                if self.current_gen > 0:
                    has_BT = True
                    has_GP = True
                if self.demonstrations is not None:
                    has_BT = True
                    has_Demo = True
                self.__continue_interaction(window, has_BT, has_GP, has_Demo)

            # region AGX Stuff
            elif ret == "__initial__":
                if not env_showing:
                    window.find_element("__initial__").update("Shutdown")
                    self.agx_GUI.disable_all_but(window, "__initial__")
                    app.GRAPHICS = True
                    sim_args = (
                        app.CloudpickleWrapper(app.Application),
                        self.agx_args,
                        self.obj_data,
                        self.initial,
                    )
                    process = proc.SimProcess(sim_args)
                    process.start()
                    env_showing = True
                else:
                    window.find_element("__initial__").update("Initial Configuration")
                    process.stop()
                    env_showing = False
                    self.__continue_interaction(window, has_BT, has_GP, has_Demo)

            elif ret == "__target__":
                if not env_showing:
                    window.find_element("__target__").update("Shutdown")
                    self.agx_GUI.disable_all_but(window, "__target__")
                    app.GRAPHICS = True
                    sim_args = (
                        app.CloudpickleWrapper(app.Application),
                        self.agx_args,
                        self.obj_data,
                        self.targets,
                    )
                    process = proc.SimProcess(sim_args)
                    process.start()
                    env_showing = True
                else:
                    window.find_element("__target__").update("Target Configuration")
                    process.stop()
                    env_showing = False
                    self.__continue_interaction(window, has_BT, has_GP, has_Demo)
            # endregion

            # region BT Stuff
            elif ret == "__run_tree__":
                if bt_running is False:
                    if self.bt is None:
                        print("No Behavior Tree to run!")
                        has_BT = False
                        self.__continue_interaction(window, has_BT, has_GP, has_Demo)
                    else:
                        # disable all elements
                        self.agx_GUI.disable_all_but(window, "__run_tree__")
                        window.find_element("__run_tree__").update("Stop")
                        print("Launching tree...")
                        bt_running = True

                        app.GRAPHICS = True
                        self.bt_process = app.RunBT(
                            self.lfd_interface,
                            self.obj_data,
                            self.behaviors,
                            self.bt,
                            self.bt_tick_freq,
                            (app.CloudpickleWrapper(app.Application), self.agx_args),
                        )
                        self.bt_process.start()

                else:
                    self.bt_process.stop()
                    self.bt_process.join()
                    bt_running = False
                    window.find_element("__run_tree__").update("Run")
                    self.__continue_interaction(window, has_BT, has_GP, has_Demo)

            elif ret == "__show_tree__":
                if os.name == "nt":  # Windows
                    os.startfile(os.path.join(self.log_folder, "best_individual.svg"))
                else:
                    opener = "xdg-open"
                    subprocess.call(
                        [opener, os.path.join(self.log_folder, "best_individual.svg")]
                    )

            elif ret == "__save_tree__":
                folder = gui.popup_get_folder("Select a folder")
                if folder is not None and folder != "":
                    if os.path.isdir(folder):
                        number = 1
                        while os.path.isdir(os.path.join(folder, f"bt{number}")):
                            number += 1

                        folder = os.path.join(folder, f"bt{number}")

                os.makedirs(folder)
                self._save_tree(self.bt, folder, self.behaviors)
                self.__continue_interaction(window, has_BT, has_GP, has_Demo)
            # endregion

            # region GP Stuff
            elif ret == "__start__":
                if not gp_running:
                    # disable all elements
                    window.find_element("__start__").update("Stop GP")
                    self.agx_GUI.disable_all_but(window, "__start__")

                    # Create behaviors
                    if not os.path.isdir(self.log_folder):
                        os.makedirs(self.log_folder)
                        self.gp_par.behavior_lists = self._generate_behaviors(
                            self.log_folder
                        )
                        self.behaviors = RobotBehaviors(
                            self.log_folder, self.task_objects
                        )
                    pytrees_params = func.get_pytree_params(
                        self.gp_par.behavior_lists, self.behaviors
                    )
                    self.gp_env = gp_environment.GPEnvironment(
                        self.gp_interface,
                        self.behaviors,
                        self.initial,
                        self.targets,
                        self.fitness_coeff,
                        pytrees_params,
                    )

                    gp_running = True
                    hotstart = True if self.current_gen > 0 else False
                    app.GRAPHICS = self.visual
                    baseline = self._get_baseline()
                    baseline_index = self.baseline_index
                    if baseline is None:
                        baseline_index = None
                    self.gp_process = gp_asprocess.GPProcess(
                        os.path.join(self.repo_path, "config"),
                        self.gp_env,
                        self.gp_par,
                        hotstart=hotstart,
                        baseline=baseline,
                        baseline_index=baseline_index,
                        visual=self.visual,
                        args=(app.CloudpickleWrapper(app.Application), self.agx_args),
                    )
                    print("Starting GP Evolution:")
                    print(f"- hotstart: {hotstart}")
                    print(f"- baseline: {baseline}")
                    print(f"- target:\n {self.targets}")
                    self.gp_process.start()

                else:
                    gp_running = False
                    has_GP = True
                    has_BT = True
                    window.find_element("__start__").update("Start GP")

                    self.gp_process.stop()
                    self.gp_process.join()
                    print(f"Stopped at: {self.current_gen + 1}.")
                    self._update_gp_values(window)
                    self.bt = self._get_baseline()
                    self.__continue_interaction(window, has_BT, has_GP, has_Demo)

            elif ret == "__plot_fitness__":
                path_to_fitness = os.path.join(self.log_folder, "Fitness.svg")
                func.display(path_to_fitness)

            elif ret == "__increase__":
                initial_n_gen = self.gp_par.n_generations
                self.gp_par.n_generations = initial_n_gen + self.current_gen + 1
                print(f"Total number of generation: {self.gp_par.n_generations}.")
                print(f"Current generation: {self.current_gen + 1}.")
            # endregion

            # region LfD Stuff
            elif ret == "__new_demo__":
                if not os.path.isdir(self.demo_folder):
                    os.makedirs(self.demo_folder)
                self._reload_demos(window, build_tree_function)
                window.disappear()
                self._new_demo(self.initial)
                self._reload_demos(window, build_tree_function)
                window.reappear()
                has_BT = True
                has_Demo = True
                self.__continue_interaction(window, has_BT, has_GP, has_Demo)

            elif ret == "__new_target__":
                if not os.path.isdir(self.demo_folder):
                    os.makedirs(self.demo_folder)
                self._reload_demos(window, build_tree_function)
                window.disappear()
                self._new_demo(self.targets)
                self._reload_demos(window, build_tree_function)
                window.reappear()
                if self.demonstrations is not None:
                    # We reached the size of the cluster so now it is safe to infer the target
                    self._add_target()
                    has_BT = True
                    has_Demo = True
                    self.__continue_interaction(window, has_BT, has_GP, has_Demo)

            elif ret == "__cluster__":
                if self.demonstrations is None:
                    _ = self._load_demo()
                plot_clusters(self.demo_folder, self.demonstrations)
                print("Clusters plotted!")
            # endregion

    def _update_gp_values(self, window) -> None:
        try:
            self.current_gen = logplot.get_state(self.gp_par.log_name)[2]
            self.best_fitness = max(logplot.get_best_fitness(self.gp_par.log_name))
            self.best_individual = logplot.get_best_individual(self.gp_par.log_name)

            window.find_element("__gen__").update(
                "Current generation: %d" % (self.current_gen + 1)
            )
            window.find_element("__fitness__").update(
                "Best fitness score: %d" % self.best_fitness
            )
            args = (
                app.CloudpickleWrapper(app.Application),
                self.agx_args,
                self.log_folder,
                self.best_individual,
                self.obj_data,
                self.behaviors,
                self.gp_interface,
            )
            app.GRAPHICS = False
            process = mp.Process(target=proc.bt_worker, args=args)
            process.start()
            time.sleep(5)
            print("Behavior Tree generated!")
        except (AttributeError, FileNotFoundError):
            window.find_element("__gen__").update(
                "Current generation: %d" % (self.current_gen + 1)
            )
            window.find_element("__fitness__").update("Best fitness score: -inf")

    def _get_baseline(self) -> List[str] or None:
        """Get the baseline from the given folder."""
        baseline = None
        try:
            with open(os.path.join(self.lfd_tree_folder, "tree.yaml")) as f:
                baseline = yaml.safe_load(f)
            print("Using baseline from demonstration!")
        except (FileNotFoundError, UnicodeDecodeError):
            print("No baseline found!")

        return baseline

    def _generate_behaviors(self, folder: str) -> BehaviorLists or None:
        """Generate behaviors for the GP parameters."""
        app.GRAPHICS = False
        process = mp.Process(
            target=proc.btlist_worker,
            args=(
                app.CloudpickleWrapper(app.Application),
                self.agx_args,
                self.obj_data,
                self.sim_data,
                folder,
                self.gp_interface,
            ),
        )
        process.start()
        time.sleep(5)
        print("Behaviors generated!")
        try:
            with open(os.path.join(folder, "BT_SETTINGS.yaml")) as f:
                settings = yaml.safe_load(f)
                behavior_list = BehaviorLists(
                    settings["fallback_nodes"],
                    settings["atomic_fallback_nodes"],
                    settings["sequence_nodes"],
                    settings["atomic_sequence_nodes"],
                    settings["condition_nodes"],
                    settings["action_nodes"],
                )
            return behavior_list
        except FileNotFoundError:
            print("Error in loading the behavior lists.")
            return None

    def _add_target(self) -> None:
        """Infer tha target from the learnt tree and add it to the config targets."""
        NUMBER_REGEX = r"[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?"
        with open(os.path.join(self.lfd_tree_folder, "tree.yaml")) as f:
            string_tree = yaml.safe_load(f)
        for j, node in enumerate(string_tree):
            if node.startswith("move"):
                match_str = (
                    f"^(move\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})"
                    + f" ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$"
                )
                match = re.match(match_str, node)
                target_obj = match[2]
                target_pose = [float(i) for i in match.group(3, 4, 5)]
                try:
                    target_pose[2] -= self.lfd_interface.gripper_offset
                except AttributeError:
                    target_pose[2] -= 0.12
                reference = match[7]
                break
            else:
                continue

        if target_obj not in list(self.targets.keys()):
            self.targets[target_obj] = {}
        self.targets[target_obj]["pose"] = target_pose
        self.targets[target_obj]["reference"] = reference
        with open(os.path.join(self.log_folder, "gp_targets.yaml"), "w+") as f:
            yaml.dump(self.targets, f, sort_keys=False)
        print(f"A new target for the item {target_obj} has been set!")

    def _new_demo(self, current_config: Dict = None) -> None:
        """Launch a new window to start a new demonstration."""
        action_GUI = GUIActions("New Demo", self.actions)
        action_GUI.initialize_layout()

        action_window = action_GUI.get_window()

        if self.demonstrations is None:
            n_demos = 0
        else:
            n_demos = self.demonstrations.n_demonstrations()

        os.mkdir(os.path.join(self.demo_folder, "demo%d" % (n_demos + 1)))

        config_path = os.path.join(self.repo_path, "config")
        app.GRAPHICS = True
        self.lfd_process = lfd_asprocess.LfDProcess(
            config_path,
            self.lfd_interface,
            (
                self.child_conn,
                app.CloudpickleWrapper(app.Application),
                self.agx_args,
                current_config,
            ),
        )
        self.lfd_process.start()

        ret = 0
        while ret is not None:
            ret, values = action_window.read()
            action_GUI.execute_action(ret, self._manipulate, n_demos)

        self.lfd_process.stop()
        self.lfd_process.join()
        print("Demonstration concluded!")

    def _manipulate(self, n_demos: int, action_name: str, gripper_state: str) -> None:
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
        if action_name == "pick":
            # Handle dropdown menu
            self.holding = get_obj_from_menu(self.pick_menu)
            self.parent_conn.send(("pick", self.holding))
            print(f"Picking {self.parent_conn.recv()}.")
            self._write_action(action_name, n_demos + 1, self.parent_conn)
        else:
            mainpulating = copy(self.holding)
            print(f"Releasing {mainpulating}.")
            self.parent_conn.send((action_name, ()))
            print(f"Holding {self.parent_conn.recv()}.")
            self.holding = None
            self._write_action(action_name, n_demos + 1, self.parent_conn)
            self.parent_conn.send(("lift", ()))
            lifted = self.parent_conn.recv()

    def _write_action(
        self, action_type: str, current_demo: str, conn: mp.connection.Connection
    ) -> None:
        """Write the pose of the end effector in all available frames as a new action."""
        action_data = {"type": action_type, "vec_pos": {}, "vec_quat": {}}
        current_demo_dir = os.path.join(self.demo_folder, "demo%d" % current_demo)
        if self.demonstrations is None:
            frames = self.all_frames
        else:
            frames = self.demonstrations.frames

        for frame in frames:
            # remove the default frame for now
            conn.send(("get_sensor_data", (self.ee_frame, frame)))
            position, orientation = conn.recv()

            action_data["vec_pos"][frame] = position.tolist()
            action_data["vec_quat"][frame] = orientation.tolist()

        action_number = (
            len(glob.glob(os.path.join(current_demo_dir, "data_*.yaml"))) + 1
        )
        with open(
            os.path.join(current_demo_dir, f"data_{action_number}.yaml"), "w"
        ) as f:
            yaml.dump(action_data, f, default_flow_style=None)

    def _reload_demos(self, window: gui.Window, build_tree_function) -> None:
        """Update the demonstration history and build the tree."""
        print("Reloading Demos...")
        n_demos = self._load_demo()
        window.find_element("__n_demos__").update(
            "Number of demonstrations: %d" % n_demos
        )
        window.find_element("__new_demo__").update(disabled=False)

        if self.demonstrations is None:
            return

        if build_tree_function is None:
            self._build_tree(window)
        else:
            build_tree_function(window)

    def _load_demo(self) -> int:
        """Load demonstration and return number of demonstrations."""
        info_file = os.path.join(self.demo_folder, "info.yaml")
        if not os.path.isfile(info_file):
            with open(info_file, "w") as f:
                yaml.dump(
                    {
                        "frames": self.all_frames,
                        "robot": self.robot_frame,
                        "default_frame": self.default_frame,
                    },
                    f,
                )
            self.demonstrations = None
            return 0
        elif len(glob.glob(os.path.join(self.demo_folder, "demo*"))) == 0:
            self.demonstrations = None
            return 0
        else:
            # Remove empty folders
            for folder in glob.glob(self.demo_folder + "/demo[0-9]*/"):
                # Each demonstration has to contain at least one action
                if not os.path.isfile(folder + "/data_1.yaml"):
                    print(f"Folder {folder} missing data file, removing.")
                    os.rmdir(folder)
            self.demonstrations = RobotDemonstration(
                self.demo_folder,
                custom_actions={
                    "pick": yumi_actions.PickAction,
                    "place": yumi_actions.PlaceAction,
                    "drop": yumi_actions.PlaceAction,
                    "move": base_actions.MoveAction,
                },
                exclude_frames={
                    "pick": [self.robot_frame, self.default_frame],
                    "place": [],
                    "drop": [],
                    "move": [self.robot_frame],
                },
            )
            return self.demonstrations.n_demonstrations()

    def _save_tree(
        self, string_tree: str, target_directory: str, behaviors: Any
    ) -> None:
        """Prepare the BT data for visualization and execution."""
        args = (
            self.child_conn,
            app.CloudpickleWrapper(app.Application),
            self.agx_args,
            target_directory,
            string_tree,
            self.obj_data,
            behaviors,
            self.lfd_interface,
        )
        app.GRAPHICS = False
        process = mp.Process(target=proc.bt_saver, args=args)
        process.start()
        self.parent_conn.send("")
        self.bt_length = self.parent_conn.recv()
        process.join()
        print("Behavior Tree generated!")

    def _get_tree_from_file(self) -> None:
        """Load an existing BT from a file."""
        # Use a learnt BT
        bt_description = os.path.join(self.lfd_tree_folder, "tree.yaml")
        try:
            with open(bt_description) as f:
                string_tree = yaml.safe_load(f)
            behaviors = RobotBehaviors(
                os.path.join(self.lfd_tree_folder, "settings"), self.task_objects
            )
            self._save_tree(string_tree, self.lfd_tree_folder, behaviors)
        except FileNotFoundError:
            print("Error in loading BT description.")

    def _learn_tree(self, pbar: gui.ProgressBar) -> str:
        """
        Prepare the Behavior Tree and return its string representation.

        The argument post_processing is a function that modifies the learnt BT.
        """
        # Learn the BT
        if not os.path.isdir(self.lfd_tree_folder):
            os.mkdir(self.lfd_tree_folder)
        settings_dir = os.path.join(self.lfd_tree_folder, "settings")

        behaviors = RobotBehaviors(settings_dir, self.task_objects)
        # Send the offline interface to the planner to expand the BT
        offline_interface = PlanningInterface(
            available_objects=self.task_objects,
            frames=self.demonstrations.frames,
            default_frame=self.demonstrations.default_frame,
            random_events=False,
            robot_frame=self.robot_frame,
        )

        equivalent = find_equivalent_actions(
            self.demonstrations,
            {
                "pick": yumi_actions.EquivalentPick,
                "place": yumi_actions.EquivalentPlace,
                "drop": yumi_actions.EquivalentPlace,
                "move": base_actions.EquivalentMove,
            },
        )

        # tree is a PyTree object!
        tree = learn_tree(
            settings_dir,
            equivalent,
            behaviors,
            offline_interface,
            iterations=50,
            callback=pbar.update_bar,
        )

        string_tree = tree.bt.bt
        print(string_tree)

        # extract the subtree moving the object and make it as a GP behavior
        if self.initial is not None:
            gp_tree = func.process_lfd_bt(
                string_tree,
                self.gp_par.behavior_lists,
                shrink=True,
                name="unstack_boxes",
                location=self.log_folder,
            )
        elif self.demonstrations.n_demonstrations() >= 3:
            gp_tree = func.process_lfd_bt(string_tree, self.gp_par.behavior_lists)

        # in subtree, the string representation of the tree is stored
        with open(os.path.join(self.lfd_tree_folder, "tree.yaml"), "w") as f:
            yaml.dump(gp_tree, f)

        self._save_tree(string_tree, self.lfd_tree_folder, behaviors)

    def _build_tree(
        self,
        window: gui.Window,
        from_file: bool = False,
    ) -> None:
        """
        Build the Behavior Tree once it is loaded.

        The argument post_processing is a function that modifies the learnt BT.
        """
        pbar = window.find_element("__progress__")
        pbar.update(visible=True)
        window.read(timeout=0)

        if from_file:
            self._get_tree_from_file()
        else:
            self._learn_tree(pbar)

        window.find_element("__n_nodes__").update(
            "Number of nodes: %d" % self.bt_length
        )
        pbar.update(visible=False)

    def __continue_interaction(
        self, window: Any, has_BT: bool, has_GP: bool, has_Demo: bool
    ) -> None:
        """Some prinouts and reset after interaction."""
        self.agx_GUI.enable_from_flags(window, has_BT, has_GP, has_Demo)
        print("Select your next functionality...")


def main() -> None:
    parser, args, leftover_args = createArgumentParser()

    gui = GUI(args)
    gui.show()


if __name__ == "__main__":
    main()
