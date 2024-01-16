import json
import os
from typing import Set, Dict, List, Any

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
action_template = {
    # * this is the name of the action. This should be unique and is used as the identifier of the action.
    "name": "move",
    # * this is the variables that the action and its conditions needs. The variables need to be grounded later.
    "entities": {
        "entity_names",  # e.g. robot desk cabinet ball ...
    },
    # * this is the mios parameters that are necessary for robot interfaces.
    "mios_parameters": None,  # this part is skipped for now
    # * this is the precondition of the action. This should be checked before the action is executed.
    # * They are explicitly implemented in the subtree as condition nodes in the sequence.
    "preconditions": {
        "true": {
            "object_name": {  # e.g. cabinet
                "property": "optional_object_name",  # e.g. open: "", on: "desk"
            },
        },
        "false": {
            "object_name": {
                "property": "optional_object_name",
            }
        },
        "propositions": Set[  # * this is for compensating the informations. should be used as less as possible.
            str
        ],
    },
    # * this is the effect of the action. This should be exerted to the world by the action
    # * node. All the effects are considered here, not only the purposes but also the side effects.
    "effects": {
        "true": {
            "object_name": {  # e.g. cabinet
                "property": "optional_object_name",  # e.g. open: "", on: "desk"
            },
        },
        "false": {
            "object_name": {
                "property": "optional_object_name",
            }
        },
        "propositions": Set[  # * this is for compensating the informations. should be used as less as possible.
            str
        ],
    },
    # * this is the reason to conduct this action. This should be checked after
    # * the action is executed, which is explicitly implemented in the subtree as a condition
    # * node in the selector.
    "purposes": {
        "true": {
            "object_name": {  # e.g. cabinet
                "property": "optional_object_name",
                # e.g. open: "", this is a property of the object
                # e.g. on: "desk" this is a directedrelation between objects
            },
        },
        "false": {
            "object_name": {
                "property": "optional_object_name",
            }
        },
        "propositions": Set[  # * this is for compensating the informations. should be used as less as possible.
            str
        ],
    },
}

# an action example
action_template = {
    # * this is the name of the action. This should be unique and is used as the identifier of the action.
    "name": "move",
    # * this is the variables that the action and its conditions needs. The variables need to be grounded later.
    "entities": {
        "entity_names",  # e.g. robot desk cabinet ball ...
    },
    # * this is the mios parameters that are necessary for robot interfaces.
    "mios_parameters": None,  # this part is skipped for now
    # * this is the precondition of the action. This should be checked before the action is executed.
    # * They are explicitly implemented in the subtree as condition nodes in the sequence.
    "preconditions": {
        "true": {
            "object_name": {  # e.g. cabinet
                "property": "optional_object_name",  # e.g. open: "", on: "desk"
            },
        },
        "false": {
            "object_name": {
                "property": "optional_object_name",
            }
        },
        "propositions": Set[  # * this is for compensating the informations. should be used as less as possible.
            str
        ],
    },
    # * this is the effect of the action. This should be exerted to the world by the action
    # * node. All the effects are considered here, not only the purposes but also the side effects.
    "effects": {
        "true": {
            "object_name": {  # e.g. cabinet
                "property": "optional_object_name",  # e.g. open: "", on: "desk"
            },
        },
        "false": {
            "object_name": {
                "property": "optional_object_name",
            }
        },
        "propositions": Set[  # * this is for compensating the informations. should be used as less as possible.
            str
        ],
    },
    # * this is the reason to conduct this action. This should be checked after
    # * the action is executed, which is explicitly implemented in the subtree as a condition
    # * node in the selector.
    "purposes": {
        "true": {
            "object_name": {  # e.g. cabinet
                "property": "optional_object_name",
                # e.g. open: "", this is a property of the object
                # e.g. on: "desk" this is a directedrelation between objects
            },
        },
        "false": {
            "object_name": {
                "property": "optional_object_name",
            }
        },
        "propositions": Set[  # * this is for compensating the informations. should be used as less as possible.
            str
        ],
    },
}


# Write variables to JSON file
actions = {
    "move": move,
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
