import json
import os

from kios_bt_planning.kios_scene.scene_factory import SceneFactory
from kios_bt_planning.kios_bt.bt_factory import BehaviorTreeFactory
from kios_bt_planning.kios_robot.robot_interface import RobotInterface
from kios_bt_planning.kios_world.world_interface import WorldInterface

####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_json_file_path = os.path.join(current_dir, "gearset_scene.json")


####################### scene
with open(scene_json_file_path, "r") as file:
    json_object = json.load(file)

gearset_scene = SceneFactory().create_scene_from_json(json_object)

####################### world
world_interface = WorldInterface()

####################### robot
robot_interface = RobotInterface()

####################### agent


####################### bt
bt_factory = BehaviorTreeFactory()
