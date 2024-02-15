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
from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import AgentResponse, KiosPromptSkeleton


from typing import List, Dict, Any
import json
import os
import logging

import py_trees

from langchain_openai import ChatOpenAI
from langchain_core.output_parsers import StrOutputParser, JsonOutputParser

# * model = "gpt-3.5-turbo-16k-0613"
# * model="gpt-4-0613", # not this
# * model="gpt-4-1106-preview",


def test_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)

    tick_loop_test(bt_stewardship)


def visualize_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)


def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    problem_name = "gearset3"  # ! updated default initial states
    problem = "(define (problem robot_assembly_problem-problem)\
                    (:domain robot_assembly_problem-domain)\
                    (:objects\
                    parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
                    gear1 gear2 gear3 shaft1 shaft2 base - part\
                    left_hand - hand\
                    )\
                    (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_inserted_to shaft2 base))\
                    (:goal (and (is_inserted_to gear3 shaft2)))\
                    )"

    # * refine problem
    # problem_name = "gearset3_sk"  # super skeleton
    # file_dir = os.path.dirname(os.path.abspath(__file__))
    # problem_dir = os.path.join(file_dir, "gearset3_cot_sk.txt")
    # with open(problem_dir, "r") as f:
    #     problem = f.read()

    file_dir = os.path.dirname(__file__)
    prompt_sk_dir = os.path.join(file_dir, "prompt_skeletons")

    llm_spt_cot_sk = KiosLLMSupporter()

    with open(os.path.join(prompt_sk_dir, "cot_skeleton.json"), "r") as f:
        cot_sk_pptsk_json = json.load(f)
        cot_sk_pptsk = KiosPromptSkeleton.from_json(cot_sk_pptsk_json)
        llm_spt_cot_sk.initialize_from_prompt_skeleton(cot_sk_pptsk)

    cot_sk_ppt = llm_spt_cot_sk.create_prompt()

    cot_sk_llm = ChatOpenAI(
        model_name=llm_spt_cot_sk.prompt_skeleton.model_name, temperature=0
    )

    chain = cot_sk_ppt | cot_sk_llm | JsonOutputParser()

    # response = chain.invoke(
    #     {
    #         "problem": problem,
    #     }
    # )

    llm_spt_re_sk = KiosLLMSupporter()

    with open(os.path.join(prompt_sk_dir, "refine_skeleton.json"), "r") as f:
        re_sk_pptsk_json = json.load(f)
        re_sk_pptsk = KiosPromptSkeleton.from_json(re_sk_pptsk_json)
        llm_spt_re_sk.initialize_from_prompt_skeleton(re_sk_pptsk)

    re_sk_ppt = llm_spt_re_sk.create_prompt()

    re_sk_llm = ChatOpenAI(
        model_name=llm_spt_re_sk.prompt_skeleton.model_name, temperature=0
    )

    re_sk_chain = re_sk_ppt | re_sk_llm | JsonOutputParser()

    # response = re_sk_chain.invoke(
    #     {
    #         "problem": problem,
    #     }
    # )

    # bt = response.get("task_plan").get("behavior_tree")

    # test_bt(bt)

    # print(response)


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
