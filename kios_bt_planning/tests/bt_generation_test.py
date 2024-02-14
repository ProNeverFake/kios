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
import os

import py_trees


def test_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
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

    # problem_name = "gearset2"  # ! updated default initial states
    # problem = "(define (problem robot_assembly_problem-problem)\
    #                 (:domain robot_assembly_problem-domain)\
    #                 (:objects\
    #                 parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
    #                 gear1 gear2 gear3 shaft1 shaft2 base - part\
    #                 left_hand - hand\
    #                 )\
    #                 (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft2 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
    #                 (:goal (and (is_inserted_to gear3 shaft2)))\
    #                 )\
    #                 "

    # problem_name = "gearset3"  # ! updated default initial states
    # problem = "(define (problem robot_assembly_problem-problem)\
    #                 (:domain robot_assembly_problem-domain)\
    #                 (:objects\
    #                 parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
    #                 gear1 gear2 gear3 shaft1 shaft2 base - part\
    #                 left_hand - hand\
    #                 )\
    #                 (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_inserted_to shaft2 base))\
    #                 (:goal (and (is_inserted_to gear3 shaft2)))\
    #                 )"

    # * refine problem
    problem_name = "gearset3_sk"  # super skeleton
    file_dir = os.path.dirname(os.path.abspath(__file__))
    problem_dir = os.path.join(file_dir, "gearset3_cot_sk.txt")
    with open(problem_dir, "r") as f:
        problem = f.read()

    llm_model = KiosLLM()

    ### * end_to_end
    # feature = "e2e"
    # model = "gpt-4-1106-preview"
    # ver = "v2"

    # prompt_dir = "prompts/end_to_end_v2"
    # prompt_load_order = [
    #     "e2e_role",  # your are a good interpreter
    #     "e2e_output_format",  # how to generate the output
    #     "e2e_domain",  # domain knowledge
    #     "e2e_problem",  # the problem format
    #     "e2e_state",  # hot to describe the state
    #     "e2e_bt",  # how to build tree
    #     "e2e_chain",  # COT
    #     "e2e_example",  # some skeleton examples ... Done
    # ]
    ### *

    ### * refine_sk
    feature = "re_sk"
    model = "gpt-4-1106-preview"
    # model = "gpt-3.5-turbo-16k-0613"
    ver = "v1"

    prompt_dir = "prompts/sk_refine"
    prompt_load_order = [
        "refine_role",  # your are a good interpreter
        "refine_input_format",  # about the input
        "refine_state",  # about the state
        # "refine_domain", # domain knowledge
        "refine_help",  # tips about how to refine the nodes
        "refine_controlflow",  # how to refine controlflow nodes
        "refine_condition",  # how to refine condition nodes
        "refine_action",  # how to refine action nodes
        "refine_output_format",  # the output format
        # "refine_chain",  # COT
    ]
    ### *

    ### *skeleton
    # feature = "skeleton"
    # model = "gpt-4-1106-preview"
    # ver = "v2"
    # prompt_dir = "prompts/skeleton"
    # prompt_load_order = [
    #     "prompt_role",  # your are a good interpreter ... Done
    #     "prompt_domain",  # domain knowledge ... Done
    #     "prompt_problem",  # how the problem is provided ... Done
    #     "prompt_environment",  # how to express the environment ... Done
    #     # "prompt_behaviortree",  # how to construct the behavior tree ... Done
    #     "prompt_bt_skeleton",  # how to construct the skeleton behavior tree ... Done
    #     "prompt_bt_skeleton_example",  # some skeleton examples ... Done
    #     "prompt_output_format",  # the output format ... Done
    #     # "prompt_example",  # some examples ... Done
    # ]
    ### *

    ### *COT spsk
    # feature = "cot_ap_skeleton"
    # model = "gpt-4-1106-preview"
    # ver = "v1"
    # """
    # add chain, try to solve state inconsistency by applying COT
    # """
    # prompt_dir = "prompts/cot_sp_skeleton"
    # prompt_load_order = [
    #     "cot_role",  # your are a good interpreter
    #     "cot_output_format",  # how to generate the output
    #     "cot_domain",  # domain knowledge
    #     "cot_problem",  # the problem format
    #     "cot_state",  # hot to describe the state
    #     "cot_bt",  # how to build tree
    #     "cot_chain",  # COT
    #     "cot_example",  # some skeleton examples ... Done
    # ]
    ### *

    # ## *COT skeleton
    # feature = "cot_skeleton"
    # model = "gpt-4-1106-preview"
    # ver = "v1"
    # """
    # add chain, try to solve state inconsistency by applying COT
    # use skeleton
    # """
    # prompt_dir = "prompts/cot_skeleton"
    # prompt_load_order = [
    #     "cot_sk_role",  # your are a good interpreter
    #     "cot_sk_output_format",  # how to generate the output
    #     "cot_sk_domain",  # domain knowledge
    #     "cot_sk_problem",  # the problem format
    #     "cot_sk_state",  # hot to describe the state
    #     "cot_sk_bt",  # how to build tree
    #     "cot_sk_chain",  # COT
    #     "cot_sk_example",  # some skeleton examples ... Done
    # ]
    # ## *

    llm_model.initialize(
        prompt_dir=prompt_dir,
        prompt_load_order=prompt_load_order,
    )

    # * model = "gpt-3.5-turbo-16k-0613"
    # * model="gpt-4-0613", # not this
    # * model="gpt-4-1106-preview",

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

# ! should the is_free has a default true? or false?
# ! can I explain the knowledge differently from the pddl domain?

# ! BUG in example: is_free is still default to be true!!!
# ! BUG in example: env after ist falsch!
# ! BUG in example: the boolean value is T F instead of t f. also should be null not None
# ! The action name "action: xxx(xxx, xxx)" is not consistent with the format requirement in action parser!
# ! BUG in output_format: four instead of three!

# ! SYSTEMATIC BUG in problem: the example problem still applies the is free to the parts!!!!!!!!
