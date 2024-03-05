import json
import os

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface
from kios_utils.pybt_test import (
    generate_bt_stewardship,
    render_dot_tree,
    tick_loop_test,
    tick_1000HZ_test,
    tick_frequency_test,
)


def core_loop():
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
    robot_interface.setup_scene(gearset_scene)

    ####################### agent
    # * use the generated plan instead

    ####################### bt
    bt_factory = BehaviorTreeFactory(
        world_interface=world_interface,
        robot_interface=robot_interface,
    )

    ####################### * old code
    # behavior_tree = None
    # with open(bt_json_file_path, "r") as file:
    #     json_object = json.load(file)
    #     behavior_tree = bt_factory.from_json_to_bt(json_object)

    # bt_steward = generate_bt_stewardship(behavior_tree)
    # render_dot_tree(bt_steward)
    # # tick_loop_test(bt_steward)
    # tick_frequency_test(bt_steward)

    # * new code
    behavior_tree_stewardship = BehaviorTreeStewardship(
        behaviortree_factory=bt_factory,
        world_interface=world_interface,
        robot_interface=robot_interface,
    )

    with open(bt_json_file_path, "r") as file:
        json_object = json.load(file)
        behavior_tree_stewardship.load_bt_json(json_object)

    behavior_tree_stewardship.generate_behavior_tree()
    behavior_tree_stewardship.setup_behavior_tree()
    behavior_tree_stewardship.render_dot_tree()

    # ! return the tree result
    behavior_tree_stewardship.tick_loop_test()

    # sim_result = behavior_tree_stewardship.simulate_behavior_tree()
    # print(sim_result.to_json())

    # ! world state has problem. gear3 is not free!


if __name__ == "__main__":
    core_loop()
