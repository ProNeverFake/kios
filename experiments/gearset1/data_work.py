import os
import json
from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree
from kios_bt.bt_factory import BehaviorTreeFactory
import datetime


def render_json_bt(bt_json: json, dir=None, name=None):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    time_stamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    render_dot_tree(
        bt_stewardship,
        name=name if name is not None else time_stamp,
        dir=dir,
    )


current_dir = os.path.dirname(__file__)

# method_dir_name = "recursive_generation_record"
method_dir_name = "one_step_record_gpt3.5"
# method_dir_name = "iterative_record"
# method_dir_name = "human_in_the_loop_generation"

# KR: NOT_NESTED

method_dir = os.path.join(current_dir, method_dir_name)

folder_count = len(
    [
        name
        for name in os.listdir(method_dir)
        if os.path.isdir(os.path.join(method_dir, name))
    ]
)

success_count = 0

for i in range(folder_count):
    folder_path = os.path.join(method_dir, str(i))
    json_file_path = os.path.join(folder_path, "result.json")
    with open(json_file_path, "r") as f:
        result = json.load(f)

    print(f'{i}: {result["tree_result"]["result"]}')
    render_json_bt(result["behavior_tree"], name="generated")
    render_json_bt(result["problem"]["result"], name="ground_truth")

    pause = input("Press Enter to continue...")


# * ONE STEP GENERATION SYSTEMATIC MISTAKE
