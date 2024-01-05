import json
import os

# an action example
move = {
    "name": "move",
    "variables": {"l_from": None, "l_to": None},
    "mios_parameters": {"parameter": None},
    "preconditions": {
        "true": {"robot_at": ["l_from"], "reachable": ["l_from", "l_to"]},
        "false": {},
    },
    "effects": {"true": {"robot_at": ["l_to"]}, "false": {"robot_at": ["l_from"]}},
}

load_tool = {
    "name": "load_tool",
    "variables": {"tool": None},
    "mios_parameters": ["something here"],
    "preconditions": {"true": {"hand_free": False}, "false": {}},
    "effects": {"true": {"in_hand": "tool"}, "false": {"hand_free": False}},
}

unload_tool = {
    "name": "unload_tool",
    "variables": {"tool": None},
    "mios_parameters": ["something here"],
    "preconditions": {"true": {"in_hand": "tool", "tool_free": "tool"}, "false": {}},
    "effects": {"true": {"hand_free": False}, "false": {"in_hand": "tool"}},
}

pick = {
    "name": "pick",
    "variables": {"part": None, "tool": None, "l_from": None, "l_to": None},
    "mios_parameters": ["something here"],
    "preconditions": {
        "true": {
            "robot_at": "l_from",
            "reachable": ["l_from", "l_to"],
            "in_hand": "tool",
            "tool_free": "tool",
        },
        "false": {},
    },
    "effects": {
        "true": {"robot_at": "l_from", "in_tool": "part"},
        "false": {"robot_at": "l_to", "tool_free": "tool"},
    },
}

# Write variables to JSON file
actions = {
    "move": move,
    "load_tool": load_tool,
    "unload_tool": unload_tool,
    "pick": pick,
}

# condition examples

is_in_hand = {
    "name": "is_in_hand",
    "variables": {"tool": None},
}

is_in_tool = {
    "name": "is_in_hand",
    "variables": {"tool": None, "part": None},
}

conditions = {
    "is_in_hand": is_in_hand,
    "is_in_tool": is_in_tool,
}

# Define the file path
# Get the current working directory
current_dir = os.getcwd()
folder_path = os.path.join(current_dir, "src/kios/kios_bt_planning/bt_settings")
# Create the folder if it doesn't exist
os.makedirs(folder_path, exist_ok=True)

actions_file_path = os.path.join(folder_path, "actions.json")

# Write variables to JSON file
with open(actions_file_path, "w") as json_file:
    json.dump(actions, json_file, indent=4)

conditions_file_path = os.path.join(folder_path, "conditions.json")

# Write variables to JSON file
with open(conditions_file_path, "w") as json_file:
    json.dump(conditions, json_file, indent=4)

# # Read variables from JSON file
# with open(actions_file_path, "r") as json_file:
#     behavior_lists = json.load(json_file)
