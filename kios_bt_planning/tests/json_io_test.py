import json
import os

# mios_parameters example
mios_parameters = {
    "skill_type": "BBCartesianMove",
    "skill_objects": {"CartesianMove": None},
    "skill_parameters": {
        "skill": {
            "objects": None,
            "time_max": 30,
            "BBCartesianMove": {
                "dX_d": [0.2, 0.2],
                "ddX_d": [0.1, 0.1],
                "DeltaX": [0, 0, 0, 0, 0, 0],
                "K_x": [1500, 1500, 1500, 600, 600, 600],
            },
        },
        "control": {"control_mode": 0},
        "user": {
            "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
            "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
        },
    },
}

# an action example
move = {
    # * this is the name of the action. This should be unique and is used as the identifier of the action.
    "name": "move",
    # * this is the variables that the action and its conditions needs. The variables need to be grounded later.
    "variables": {"l_from": None, "l_to": None},
    # * this is the mios parameters that are necessary for robot interfaces.
    # * the necessary "objects" are grounded from the variables.
    "mios_parameters": {
        "skill_type": "BBCartesianMove",
        "skill_objects": {"CartesianMove": "l_to"},
        "skill_parameters": {
            "skill": {
                "objects": None,
                "time_max": 30,
                "BBCartesianMove": {
                    "dX_d": [0.2, 0.2],
                    "ddX_d": [0.1, 0.1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
            },
            "control": {"control_mode": 0},
            "user": {
                "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
                "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
            },
        },
    },
    # * this is the precondition of the action. This should be checked before the action is executed.
    # * They are explicitly implemented in the subtree as condition nodes in the sequence.
    "preconditions": {
        "true": {  # * these are the conditions that need to be true for the action to be executed.
            "robot_at": {"l_from": None},
            "reachable": {"l_from": None, "l_to": None},
        },
        "false": {},  # * need to be false
    },
    # * this is the effect of the action. This should be exerted to the world by the action
    # * node. All the effects are considered here, not only the purposes but also the side effects.
    "effects": {
        "true": {"robot_at": {"l_to": None}},
        "false": {
            "robot_at": {  # * this is the side effect, or so to say, the effect that is not intended.
                "l_from": None
            }
        },
    },
    # * this is the reason to conduct this action. This should be checked after
    # * the action is executed, which is explicitly implemented in the subtree as a condition
    # * node in the selector.
    "purposes": {
        "true": {"robot_at": {"l_to": None}},  # * easy to understand
        "false": {},  # * "false" purpose, for example, you need to remove somthing on the road to pass.
    },
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
