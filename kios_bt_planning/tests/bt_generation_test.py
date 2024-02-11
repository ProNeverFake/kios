from kios_bt.data_types import Action, Condition, ObjectProperty, ControlFlow

from kios_bt.behavior_nodes import ActionNode, ConditionNode, ActionNodeTest

from kios_world.world_interface import WorldInterface

from kios_bt.bt_factory import BehaviorTreeFactory

from kios_bt.bt_stewardship import BehaviorTreeStewardship

from kios_utils.pybt_test import (
    generate_bt_stewardship,
    tick_once_test,
    render_dot_tree,
    tick_loop_test,
)

from kios_agent.kios_llm import KiosLLM

from typing import List, Dict, Any

import json

import py_trees


def test_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)

    tick_loop_test(bt_stewardship)


# def test_llm():
#     problem_name = "gearset_skeleton_v1"
#     problem = "(define (problem robot_assembly_problem-problem)\
#                     (:domain robot_assembly_problem-domain)\
#                     (:objects\
#                         parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
#                         gear1 gear2 gear3 shaft1 shaft2 base - part\
#                         left_hand - hand\
#                     )\
#                     (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft1 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
#                     (:goal (and (is_inserted_to gear1 shaft1)))\
#                     )"  # ! CHEAT

#     llm_model = KiosLLM()
#     llm_model.initialize()

#     # run the instructions one by one
#     response = llm_model.query_llm(problem, problem_name)


def visualize_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)

    # tick_loop_test(bt_stewardship)


def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # problem_name = "gearset_problem1_skeleton_v1"
    # problem = "(define (problem robot_assembly_problem-problem)\
    #                 (:domain robot_assembly_problem-domain)\
    #                 (:objects\
    #                     parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
    #                     gear1 gear2 gear3 shaft1 shaft2 base - part\
    #                     left_hand - hand\
    #                 )\
    #                 (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft1 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
    #                 (:goal (and (is_inserted_to gear1 shaft1)))\
    #                 )"  # ! CHEAT

    problem_name = "gearset2"
    problem = "(define (problem robot_assembly_problem-problem)\
                    (:domain robot_assembly_problem-domain)\
                    (:objects\
                    parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
                    gear1 gear2 gear3 shaft1 shaft2 base - part\
                    left_hand - hand\
                    )\
                    (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft2 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
                    (:goal (and (is_inserted_to gear3 shaft2)))\
                    )\
                    "

    llm_model = KiosLLM()
    llm_model.initialize()

    model = "gpt-3.5-turbo-16k-0613"
    # model="gpt-4-0613", # not this
    # model="gpt-4-1106-preview",
    feature = "skeleton"
    # model = "gpt-4-1106-preview"
    ver = "v2"

    full_problem_name = f"{problem_name}_{feature}_{ver}_{model}"

    response = llm_model.query_llm(
        problem=problem,
        problem_name=full_problem_name,
        model=model,
    )

    bt = response["task_plan"]["behavior_tree"]

    test_bt(bt)


if __name__ == "__main__":
    main()

    # test_bt(result_bt_json)
    # test_bt(bt_skeleton)
    # visualize_bt(bt_skeleton)
