import json
import os

from kios_bt_planning.kios_scene.scene_factory import SceneFactory
from kios_bt_planning.kios_bt.bt_factory import BehaviorTreeFactory
from kios_bt_planning.kios_robot.robot_interface import RobotInterface
from kios_bt_planning.kios_world.world_interface import WorldInterface

####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_json_file_path = os.path.join(current_dir, "gearset_scene.json")
bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_json_file_path = os.path.join(current_dir, "world_state.json")

####################### scene
with open(scene_json_file_path, "r") as file:
    json_object = json.load(file)

gearset_scene = SceneFactory().create_scene_from_json(json_object)

####################### world
world_interface = WorldInterface()
with open(world_state_json_file_path, "r") as file:
    json_object = json.load(file)
    world_interface.load_world_from_json(json_object)


####################### robot
robot_interface = RobotInterface(
    robot_address="127.0.0.1",
    robot_port=12000,
)

####################### agent
# * use the generated plan instead


####################### bt
bt_factory = BehaviorTreeFactory(
    world_interface=world_interface,
    robot_interface=robot_interface,
)

behavior_tree = None
with open(bt_json_file_path, "r") as file:
    json_object = json.load(file)
    behavior_tree = bt_factory.from_json_to_bt(json_object)
