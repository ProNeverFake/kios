"""
node expanding method script. 
run the algo to generate the tree for current goal, forecast the future world state, and find the new goal to expand.
currently not a langgraph.
"""

import json
import os
from pprint import pprint
import operator
import logging
import re

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "default"


from kios_bt.bt_factory import BehaviorTreeFactory


from kios_agent.kios_graph import rec_ut_generator_chain_ft, rec_ut_generator_chain_gpt3

from dotenv import load_dotenv

from langsmith import traceable

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree
import datetime

timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

current_dir = os.path.dirname(os.path.abspath(__file__))

problem_number = 5
action = "pick_up(left_hand, outwardgripper, lampshade)"

result_file = os.path.join(current_dir, str(problem_number), "gpt3.txt")

metadata = {
    "method_name": "ut",
    "try_count": problem_number,
    "timestamp": timestamp,
    "usecase": "chair",
}


def render_bt(bt_json: json, dir=None):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    time_stamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    render_dot_tree(bt_stewardship, name=time_stamp, dir=dir)


def write_result(bt_json: json, dir: str):
    with open(dir, "w") as f:
        json.dump(bt_json, f)


# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())

prompt_dir = os.path.join(data_dir, "prompts")


@traceable(name="rec_unit_subtree", metadata=metadata)
def ut_gen_test():
    ut = rec_ut_generator_chain_gpt3.invoke(
        {
            "action": action,
        }
    )

    vis_dir = os.path.join(current_dir, str(problem_number), "gpt3")
    if not os.path.exists(vis_dir):
        os.makedirs(vis_dir)

    render_bt(ut, vis_dir)

    write_result(ut, result_file)


if __name__ == "__main__":
    ut_gen_test()
    pass
