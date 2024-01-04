from dynamic_bt.behaviors import behavior_list_test_settings as test_settings

import os, yaml
from dynamic_bt.behaviors.behavior_lists import BehaviorLists
import os

if __name__ == "__main__":
    print(test_settings.get_condition_nodes())
    print(test_settings.get_action_nodes())

    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, "BT_SETTINGS.yaml")

    with open(file_path) as f:
        # Rest of your code here
        settings = yaml.safe_load(f)
        behavior_list = BehaviorLists(
            settings["fallback_nodes"],
            settings["atomic_fallback_nodes"],
            settings["sequence_nodes"],
            settings["atomic_sequence_nodes"],
            settings["condition_nodes"],
            settings["action_nodes"],
        )

    print("behavior_list.condition_nodes: %s" % behavior_list.condition_nodes)
    print("behavior_list.action_nodes: %s" % behavior_list.action_nodes)
    print("behavior_list.sequence_nodes: %s" % behavior_list.sequence_nodes)
    print("behavior_list.fallback_nodes: %s" % behavior_list.fallback_nodes)
    print(
        "behavior_list.atomic_sequence_nodes: %s" % behavior_list.atomic_sequence_nodes
    )
    print(
        "behavior_list.atomic_fallback_nodes: %s" % behavior_list.atomic_fallback_nodes
    )
