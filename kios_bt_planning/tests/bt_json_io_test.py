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

from typing import List, Dict, Any

import json

import py_trees


def test_simple_bt():
    bt_json = {
        "name": "Pick Up Apple",
        "identifier": 0,
        "type_name": "selector",
        "children": [
            {
                "summary": "check if the apple is in hand",
                "name": "check apple in hand",
                "identifier": 1,
                "type_name": "condition",
                "conditions": [
                    {
                        "object_name": "apple",
                        "property_name": "in",
                        "property_value": "hand",
                        "status": True,
                    }
                ],
            },
            {
                "name": "Pick Up Sequence",
                "identifier": 2,
                "type_name": "sequence",
                "children": [
                    {
                        "summary": "check if the apple is on the ground",
                        "name": "check apple on the ground",
                        "identifier": 3,
                        "type_name": "condition",
                        "conditions": [
                            {
                                "object_name": "apple",
                                "property_name": "on_the_ground",
                                "property_value": None,
                                "status": True,
                            }
                        ],
                    },
                    {
                        "summary": "check if the hand is free",
                        "name": "check hand free",
                        "identifier": 4,
                        "type_name": "condition",
                        "conditions": [
                            {
                                "object_name": "hand",
                                "property_name": "free",
                                "property_value": None,
                                "status": True,
                            }
                        ],
                    },
                    {
                        "summary": "pick up the apple",
                        "name": "pick_up",
                        "identifier": 5,
                        "type_name": "action",
                        "effects": [
                            {
                                "object_name": "apple",
                                "property_name": "on_the_ground",
                                "property_value": None,
                                "status": False,
                            },
                            {
                                "object_name": "apple",
                                "property_name": "in",
                                "property_value": "hand",
                                "status": True,
                            },
                            {
                                "object_name": "hand",
                                "property_name": "free",
                                "property_value": None,
                                "status": False,
                            },
                        ],
                    },
                ],
            },
        ],
    }

    # be_stewardship = BehaviorTreeStewardship()

    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_bt(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)

    tick_loop_test(bt_stewardship)


example_json_data = {
    "name": "load_tool selector",
    "identifier": 0,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if parallel_box1 is hold",
            "name": "check left_hand hold parallel_box1",
            "identifier": 1,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "left_hand",
                    "property_name": "hold",
                    "property_value": "parallel_box1",
                    "status": True,
                }
            ],
        },
        {
            "name": "load_tool Sequence",
            "identifier": 2,
            "type_name": "sequence",
            "children": [
                {
                    "summary": "check if parallel_box1 is equippable",
                    "name": "check parallel_box1 is_equippable",
                    "identifier": 3,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_equippable",
                            "property_value": None,
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "check if left hand is free",
                    "name": "check left_hand is_free",
                    "identifier": 4,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "left_hand",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "equip parallel_box1 to left hand",
                    "name": "load_tool(left_hand, parallel_box1)",
                    "identifier": 5,
                    "type_name": "action",
                    "effects": [
                        {
                            "object_name": "hand",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": False,
                        },
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_equippable",
                            "property_value": None,
                            "status": False,
                        },
                        {
                            "object_name": "hand",
                            "property_name": "hold",
                            "property_value": "parallel_box1",
                            "status": True,
                        },
                    ],
                },
            ],
        },
    ],
}

example_json_data = {
    "name": "pick_up selector",
    "identifier": 6,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if parallel_box1 holds gear1",
            "name": "check parallel_box1 hold gear1",
            "identifier": 7,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "parallel_box1",
                    "property_name": "hold",
                    "property_value": "gear1",
                    "status": True,
                }
            ],
        },
        {
            "name": "pick_up Sequence",
            "identifier": 8,
            "type_name": "sequence",
            "children": [
                {
                    "summary": "check if parallel_box1 is free",
                    "name": "check parallel_box1 is_free",
                    "identifier": 9,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "check if left hand holds parallel_box1",
                    "name": "check left_hand hold parallel_box1",
                    "identifier": 10,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "left_hand",
                            "property_name": "hold",
                            "property_value": "parallel_box1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "check if parallel_box1 can manipulate gear1",
                    "name": "check left_hand hold parallel_box1",
                    "identifier": 11,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "left_hand",
                            "property_name": "hold",
                            "property_value": "parallel_box1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "pick up gear1 using parallel_box1",
                    "name": "pick_up(left_hand, parallel_box1, gear1)",
                    "identifier": 12,
                    "type_name": "action",
                    "effects": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "hold",
                            "property_value": "gear1",
                            "status": True,
                        },
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": False,
                        },
                    ],
                },
            ],
        },
    ],
}

this_example_json_data = {
    "name": "insert selector",
    "identifier": 13,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if gear1 is inserted to shaft1",
            "name": "check gear1 is_inserted_to shaft1",
            "identifier": 14,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "gear1",
                    "property_name": "is_inserted_to",
                    "property_value": "shaft1",
                    "status": True,
                }
            ],
        },
        {
            "name": "insert sequence",
            "identifier": 15,
            "type_name": "sequence",
            "children": [
                {
                    "summary": "check if left hand holds parallel_box1",
                    "name": "check left_hand hold parallel_box1",
                    "identifier": 16,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "left_hand",
                            "property_name": "hold",
                            "property_value": "parallel_box1",
                            "status": True,
                        }
                    ],
                },
                ################
                {
                    "name": "pick_up selector",
                    "identifier": 6,
                    "type_name": "selector",
                    "children": [
                        {
                            "summary": "check if parallel_box1 holds gear1",
                            "name": "check parallel_box1 hold gear1",
                            "identifier": 7,
                            "type_name": "condition",
                            "conditions": [
                                {
                                    "object_name": "parallel_box1",
                                    "property_name": "hold",
                                    "property_value": "gear1",
                                    "status": True,
                                }
                            ],
                        },
                        {
                            "name": "pick_up Sequence",
                            "identifier": 8,
                            "type_name": "sequence",
                            "children": [
                                {
                                    "summary": "check if parallel_box1 is free",
                                    "name": "check parallel_box1 is_free",
                                    "identifier": 9,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                ################
                                {
                                    "name": "load_tool selector",
                                    "identifier": 0,
                                    "type_name": "selector",
                                    "children": [
                                        {
                                            "summary": "check if parallel_box1 is hold",
                                            "name": "check left_hand hold parallel_box1",
                                            "identifier": 1,
                                            "type_name": "condition",
                                            "conditions": [
                                                {
                                                    "object_name": "left_hand",
                                                    "property_name": "hold",
                                                    "property_value": "parallel_box1",
                                                    "status": True,
                                                }
                                            ],
                                        },
                                        {
                                            "name": "load_tool Sequence",
                                            "identifier": 2,
                                            "type_name": "sequence",
                                            "children": [
                                                {
                                                    "summary": "check if parallel_box1 is equippable",
                                                    "name": "check parallel_box1 is_equippable",
                                                    "identifier": 3,
                                                    "type_name": "condition",
                                                    "conditions": [
                                                        {
                                                            "object_name": "parallel_box1",
                                                            "property_name": "is_equippable",
                                                            "property_value": None,
                                                            "status": True,
                                                        }
                                                    ],
                                                },
                                                {
                                                    "summary": "check if left hand is free",
                                                    "name": "check left_hand is_free",
                                                    "identifier": 4,
                                                    "type_name": "condition",
                                                    "conditions": [
                                                        {
                                                            "object_name": "left_hand",
                                                            "property_name": "is_free",
                                                            "property_value": None,
                                                            "status": True,
                                                        }
                                                    ],
                                                },
                                                {
                                                    "summary": "equip parallel_box1 to left hand",
                                                    "name": "load_tool(left_hand, parallel_box1)",
                                                    "identifier": 5,
                                                    "type_name": "action",
                                                    "effects": [
                                                        {
                                                            "object_name": "left_hand",
                                                            "property_name": "is_free",
                                                            "property_value": None,
                                                            "status": False,
                                                        },
                                                        {
                                                            "object_name": "parallel_box1",
                                                            "property_name": "is_equippable",
                                                            "property_value": None,
                                                            "status": False,
                                                        },
                                                        {
                                                            "object_name": "left_hand",
                                                            "property_name": "hold",
                                                            "property_value": "parallel_box1",
                                                            "status": True,
                                                        },
                                                    ],
                                                },
                                            ],
                                        },
                                    ],
                                },
                                ################
                                {
                                    "summary": "check if parallel_box1 can manipulate gear1",
                                    "name": "check left_hand hold parallel_box1",
                                    "identifier": 11,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "left_hand",
                                            "property_name": "hold",
                                            "property_value": "parallel_box1",
                                            "status": True,
                                        }
                                    ],
                                },
                                {
                                    "summary": "pick up gear1 using parallel_box1",
                                    "name": "pick_up(left_hand, parallel_box1, gear1)",
                                    "identifier": 12,
                                    "type_name": "action",
                                    "effects": [
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "hold",
                                            "property_value": "gear1",
                                            "status": True,
                                        },
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": False,
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                ################
                {
                    "summary": "check if gear1 can be inserted to shaft1",
                    "name": "check gear1 can_insert_to shaft1",
                    "identifier": 18,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "gear1",
                            "property_name": "can_insert_to",
                            "property_value": "shaft1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "insert gear1 to shaft1",
                    "name": "insert(left_hand, parallel_box1, gear1, shaft1)",
                    "identifier": 19,
                    "type_name": "action",
                    "effects": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "hold",
                            "property_value": "gear1",
                            "status": False,
                        },
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": True,
                        },
                        {
                            "object_name": "gear1",
                            "property_name": "is_inserted_to",
                            "property_value": "shaft1",
                            "status": True,
                        },
                    ],
                },
            ],
        },
    ],
}

example_json_data = {
    "name": "insert selector",
    "identifier": 13,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if gear1 is inserted to shaft1",
            "name": "check gear1 is_inserted_to shaft1",
            "identifier": 14,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "gear1",
                    "property_name": "is_inserted_to",
                    "property_value": "shaft1",
                    "status": True,
                }
            ],
        },
        {
            "name": "insert sequence",
            "identifier": 15,
            "type_name": "sequence",
            "children": [
                {
                    "summary": "check if left hand holds parallel_box1",
                    "name": "check left_hand hold parallel_box1",
                    "identifier": 16,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "left_hand",
                            "property_name": "hold",
                            "property_value": "parallel_box1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "check if parallel_box1 holds gear1",
                    "name": "check parallel_box1 hold gear1",
                    "identifier": 17,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "hold",
                            "property_value": "gear1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "check if gear1 can be inserted to shaft1",
                    "name": "check gear1 can_insert_to shaft1",
                    "identifier": 18,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "gear1",
                            "property_name": "can_insert_to",
                            "property_value": "shaft1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "insert gear1 to shaft1",
                    "name": "insert(left_hand, parallel_box1, gear1, shaft1)",
                    "identifier": 19,
                    "type_name": "action",
                    "effects": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "hold",
                            "property_value": "gear1",
                            "status": False,
                        },
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": True,
                        },
                        {
                            "object_name": "gear1",
                            "property_name": "is_inserted_to",
                            "property_value": "shaft1",
                            "status": True,
                        },
                    ],
                },
            ],
        },
    ],
}

result_bt_json = {
    "name": "insert gear3 into shaft2 selector",
    "identifier": 0,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if gear3 is inserted to shaft2",
            "name": "check gear3 is_inserted_to shaft2",
            "identifier": 1,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "gear3",
                    "property_name": "is_inserted_to",
                    "property_value": "shaft2",
                    "status": True,
                }
            ],
        },
        {
            "name": "insert gear3 into shaft2 sequence",
            "identifier": 2,
            "type_name": "sequence",
            "children": [
                {
                    "name": "load_tool selector",
                    "identifier": 3,
                    "type_name": "selector",
                    "children": [
                        {
                            "summary": "check if inward_claw is held by left hand",
                            "name": "check left_hand hold inward_claw",
                            "identifier": 4,
                            "type_name": "condition",
                            "conditions": [
                                {
                                    "object_name": "left_hand",
                                    "property_name": "hold",
                                    "property_value": "inward_claw",
                                    "status": True,
                                }
                            ],
                        },
                        {
                            "name": "load_tool sequence",
                            "identifier": 5,
                            "type_name": "sequence",
                            "children": [
                                {
                                    "summary": "check if inward_claw is equippable",
                                    "name": "check inward_claw is_equippable",
                                    "identifier": 6,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "inward_claw",
                                            "property_name": "is_equippable",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                {
                                    "summary": "check if left hand is free",
                                    "name": "check left_hand is_free",
                                    "identifier": 7,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "left_hand",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                {
                                    "summary": "equip inward_claw to left hand",
                                    "name": "load_tool(left_hand, inward_claw)",
                                    "identifier": 8,
                                    "type_name": "action",
                                    "effects": [
                                        {
                                            "object_name": "left_hand",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": False,
                                        },
                                        {
                                            "object_name": "inward_claw",
                                            "property_name": "is_equippable",
                                            "property_value": None,
                                            "status": False,
                                        },
                                        {
                                            "object_name": "left_hand",
                                            "property_name": "hold",
                                            "property_value": "inward_claw",
                                            "status": True,
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "name": "pickup selector",
                    "identifier": 9,
                    "type_name": "selector",
                    "children": [
                        {
                            "summary": "check if gear3 is held by inward_claw",
                            "name": "check inward_claw hold gear3",
                            "identifier": 10,
                            "type_name": "condition",
                            "conditions": [
                                {
                                    "object_name": "inward_claw",
                                    "property_name": "hold",
                                    "property_value": "gear3",
                                    "status": True,
                                }
                            ],
                        },
                        {
                            "name": "pickup sequence",
                            "identifier": 11,
                            "type_name": "sequence",
                            "children": [
                                {
                                    "summary": "check if gear3 is free",
                                    "name": "check gear3 is_free",
                                    "identifier": 12,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "gear3",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                {
                                    "summary": "pick up gear3 using inward_claw",
                                    "name": "pickup(left_hand, gear3, inward_claw)",
                                    "identifier": 13,
                                    "type_name": "action",
                                    "effects": [
                                        {
                                            "object_name": "inward_claw",
                                            "property_name": "hold",
                                            "property_value": "gear3",
                                            "status": True,
                                        },
                                        {
                                            "object_name": "inward_claw",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": False,
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "check if gear3 can be inserted to shaft2",
                    "name": "check gear3 can_insert_to shaft2",
                    "identifier": 14,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "gear3",
                            "property_name": "can_insert_to",
                            "property_value": "shaft2",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "insert gear3 into shaft2",
                    "name": "insert(left_hand, inward_claw, gear3, shaft2)",
                    "identifier": 15,
                    "type_name": "action",
                    "effects": [
                        {
                            "object_name": "inward_claw",
                            "property_name": "hold",
                            "property_value": "gear3",
                            "status": False,
                        },
                        {
                            "object_name": "inward_claw",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": True,
                        },
                        {
                            "object_name": "gear3",
                            "property_name": "is_inserted_to",
                            "property_value": "shaft2",
                            "status": True,
                        },
                    ],
                },
            ],
        },
    ],
}

prompt_example = {
    "name": "insert selector",
    "identifier": 13,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if gear1 is inserted to shaft1",
            "name": "check gear1 is_inserted_to shaft1",
            "identifier": 14,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "gear1",
                    "property_name": "is_inserted_to",
                    "property_value": "shaft1",
                    "status": True,
                }
            ],
        },
        {
            "name": "insert sequence",
            "identifier": 15,
            "type_name": "sequence",
            "children": [
                {
                    "name": "load_tool selector",
                    "identifier": 0,
                    "type_name": "selector",
                    "children": [
                        {
                            "summary": "check if parallel_box1 is hold",
                            "name": "check left_hand hold parallel_box1",
                            "identifier": 1,
                            "type_name": "condition",
                            "conditions": [
                                {
                                    "object_name": "left_hand",
                                    "property_name": "hold",
                                    "property_value": "parallel_box1",
                                    "status": True,
                                }
                            ],
                        },
                        {
                            "name": "load_tool Sequence",
                            "identifier": 2,
                            "type_name": "sequence",
                            "children": [
                                {
                                    "summary": "check if parallel_box1 is equippable",
                                    "name": "check parallel_box1 is_equippable",
                                    "identifier": 3,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "is_equippable",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                {
                                    "summary": "check if left hand is free",
                                    "name": "check left_hand is_free",
                                    "identifier": 4,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "left_hand",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                {
                                    "summary": "equip parallel_box1 to left hand",
                                    "name": "load_tool(left_hand, parallel_box1)",
                                    "identifier": 5,
                                    "type_name": "action",
                                    "effects": [
                                        {
                                            "object_name": "hand",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": False,
                                        },
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "is_equippable",
                                            "property_value": None,
                                            "status": False,
                                        },
                                        {
                                            "object_name": "hand",
                                            "property_name": "hold",
                                            "property_value": "parallel_box1",
                                            "status": True,
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                ################
                {
                    "name": "pick_up selector",
                    "identifier": 6,
                    "type_name": "selector",
                    "children": [
                        {
                            "summary": "check if parallel_box1 holds gear1",
                            "name": "check parallel_box1 hold gear1",
                            "identifier": 7,
                            "type_name": "condition",
                            "conditions": [
                                {
                                    "object_name": "parallel_box1",
                                    "property_name": "hold",
                                    "property_value": "gear1",
                                    "status": True,
                                }
                            ],
                        },
                        {
                            "name": "pick_up Sequence",
                            "identifier": 8,
                            "type_name": "sequence",
                            "children": [
                                {
                                    "summary": "check if parallel_box1 is free",
                                    "name": "check parallel_box1 is_free",
                                    "identifier": 9,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": True,
                                        }
                                    ],
                                },
                                ################
                                {
                                    "summary": "check if left hand holds parallel_box1",
                                    "name": "check left_hand hold parallel_box1",
                                    "identifier": 16,
                                    "type_name": "condition",
                                    "conditions": [
                                        {
                                            "object_name": "left_hand",
                                            "property_name": "hold",
                                            "property_value": "parallel_box1",
                                            "status": True,
                                        }
                                    ],
                                },
                                ################
                                {
                                    "summary": "pick up gear1 using parallel_box1",
                                    "name": "pick_up(left_hand, parallel_box1, gear1)",
                                    "identifier": 12,
                                    "type_name": "action",
                                    "effects": [
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "hold",
                                            "property_value": "gear1",
                                            "status": True,
                                        },
                                        {
                                            "object_name": "parallel_box1",
                                            "property_name": "is_free",
                                            "property_value": None,
                                            "status": False,
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                ################
                {
                    "summary": "check if gear1 can be inserted to shaft1",
                    "name": "check gear1 can_insert_to shaft1",
                    "identifier": 18,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "gear1",
                            "property_name": "can_insert_to",
                            "property_value": "shaft1",
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "insert gear1 to shaft1",
                    "name": "insert(left_hand, parallel_box1, gear1, shaft1)",
                    "identifier": 19,
                    "type_name": "action",
                    "effects": [
                        {
                            "object_name": "parallel_box1",
                            "property_name": "hold",
                            "property_value": "gear1",
                            "status": False,
                        },
                        {
                            "object_name": "parallel_box1",
                            "property_name": "is_free",
                            "property_value": None,
                            "status": True,
                        },
                        {
                            "object_name": "gear1",
                            "property_name": "is_inserted_to",
                            "property_value": "shaft1",
                            "status": True,
                        },
                    ],
                },
            ],
        },
    ],
}


def test_result_bt(result_bt):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_bt(result_bt)
    bt_stewardship = generate_bt_stewardship(bt)
    bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)

    tick_loop_test(bt_stewardship)


def test_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_bt(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)

    tick_loop_test(bt_stewardship)


if __name__ == "__main__":
    # test_simple_bt()
    # test_result_bt(result_bt_json)
    # test_result_bt(result_bt_json)
    # test_bt(prompt_example)
    test_bt(result_bt_json)
