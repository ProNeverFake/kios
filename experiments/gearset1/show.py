result_1 = {
    "behavior_tree": {
        "summary": "selector to insert gear2 into shaft2",
        "name": "selector: insert(left_hand, outward_claw, gear2, shaft2)",
        "identifier": 1,
        "type_name": "selector",
        "children": [
            {
                "summary": "condition node to check if gear2 is inserted to shaft2",
                "name": "target: is_inserted_to(gear2, shaft2)",
                "identifier": 2,
                "type_name": "condition",
                "conditions": [
                    {
                        "object_name": "gear2",
                        "property_name": "is_inserted_to",
                        "property_value": "shaft2",
                        "status": True,
                    }
                ],
            },
            {
                "summary": "sequence to insert gear2 into shaft2",
                "name": "sequence: insert(left_hand, outward_claw, gear2, shaft2)",
                "identifier": 3,
                "type_name": "sequence",
                "children": [
                    {
                        "summary": "selector to load outward_claw",
                        "name": "selector: load_tool(left_hand, outward_claw)",
                        "identifier": 4,
                        "type_name": "selector",
                        "children": [
                            {
                                "summary": "condition node to check if left_hand holds outward_claw",
                                "name": "target: hold(left_hand, outward_claw)",
                                "identifier": 5,
                                "type_name": "condition",
                                "conditions": [
                                    {
                                        "object_name": "left_hand",
                                        "property_name": "hold",
                                        "property_value": "outward_claw",
                                        "status": True,
                                    }
                                ],
                            },
                            {
                                "summary": "sequence to load outward_claw",
                                "name": "sequence: load_tool(left_hand, outward_claw)",
                                "identifier": 6,
                                "type_name": "sequence",
                                "children": [
                                    {
                                        "summary": "condition node to check if outward_claw is equippable",
                                        "name": "precondition: is_equippable(outward_claw)",
                                        "identifier": 7,
                                        "type_name": "condition",
                                        "conditions": [
                                            {
                                                "object_name": "outward_claw",
                                                "property_name": "is_equippable",
                                                "property_value": None,
                                                "status": True,
                                            }
                                        ],
                                    },
                                    {
                                        "summary": "condition node to check if left hand is free",
                                        "name": "precondition: is_free(left_hand)",
                                        "identifier": 8,
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
                                        "summary": "action node to equip outward_claw to left hand",
                                        "name": "action: load_tool(left_hand, outward_claw)",
                                        "identifier": 9,
                                        "type_name": "action",
                                        "effects": [
                                            {
                                                "object_name": "left_hand",
                                                "property_name": "is_free",
                                                "property_value": None,
                                                "status": False,
                                            },
                                            {
                                                "object_name": "outward_claw",
                                                "property_name": "is_equippable",
                                                "property_value": None,
                                                "status": False,
                                            },
                                            {
                                                "object_name": "left_hand",
                                                "property_name": "hold",
                                                "property_value": "outward_claw",
                                                "status": True,
                                            },
                                        ],
                                    },
                                ],
                            },
                        ],
                    },
                    {
                        "summary": "selector to use left_hand with outward_claw to pick up gear2",
                        "name": "selector: pick_up(left_hand, outward_claw, gear2)",
                        "identifier": 10,
                        "type_name": "selector",
                        "children": [
                            {
                                "summary": "condition node to check if outward_claw holds gear2",
                                "name": "target: hold(outward_claw, gear2)",
                                "identifier": 11,
                                "type_name": "condition",
                                "conditions": [
                                    {
                                        "object_name": "outward_claw",
                                        "property_name": "hold",
                                        "property_value": "gear2",
                                        "status": True,
                                    }
                                ],
                            },
                            {
                                "summary": "sequence to use left hand with outward_claw to pick up gear2",
                                "name": "sequence: pick_up(left_hand, outward_claw, gear2)",
                                "identifier": 12,
                                "type_name": "sequence",
                                "children": [
                                    {
                                        "summary": "condition node to check if outward_claw is free",
                                        "name": "precondition: is_free(outward_claw)",
                                        "identifier": 13,
                                        "type_name": "condition",
                                        "conditions": [
                                            {
                                                "object_name": "outward_claw",
                                                "property_name": "is_free",
                                                "property_value": None,
                                                "status": True,
                                            }
                                        ],
                                    },
                                    {
                                        "summary": "condition node to check if left hand holds outward_claw",
                                        "name": "precondition: hold(left_hand, outward_claw)",
                                        "identifier": 14,
                                        "type_name": "condition",
                                        "conditions": [
                                            {
                                                "object_name": "left_hand",
                                                "property_name": "hold",
                                                "property_value": "outward_claw",
                                                "status": True,
                                            }
                                        ],
                                    },
                                    {
                                        "summary": "action node to use left hand with outward_claw to pick up gear2",
                                        "name": "action: pick_up(left_hand, outward_claw, gear2)",
                                        "identifier": 15,
                                        "type_name": "action",
                                        "effects": [
                                            {
                                                "object_name": "outward_claw",
                                                "property_name": "hold",
                                                "property_value": "gear2",
                                                "status": True,
                                            },
                                            {
                                                "object_name": "outward_claw",
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
                        "summary": "condition node to check if gear2 can be inserted to shaft2",
                        "name": "precondition: can_insert_to(gear2, shaft2)",
                        "identifier": 16,
                        "type_name": "condition",
                        "conditions": [
                            {
                                "object_name": "gear2",
                                "property_name": "can_insert_to",
                                "property_value": "shaft2",
                                "status": True,
                            }
                        ],
                    },
                    {
                        "summary": "action node to use left_hand with outward_claw to insert gear2 to shaft2",
                        "name": "action: insert(left_hand, outward_claw, gear2, shaft2)",
                        "identifier": 17,
                        "type_name": "action",
                        "effects": [
                            {
                                "object_name": "outward_claw",
                                "property_name": "hold",
                                "property_value": "gear2",
                                "status": False,
                            },
                            {
                                "object_name": "outward_claw",
                                "property_name": "is_free",
                                "property_value": None,
                                "status": True,
                            },
                            {
                                "object_name": "gear2",
                                "property_name": "is_inserted_to",
                                "property_value": "shaft2",
                                "status": True,
                            },
                        ],
                    },
                ],
            },
        ],
    },
    "world_state": [
        {
            "objects": [
                {"name": "parallel_box1", "properties": ["is_free"]},
                {"name": "shaft1", "properties": []},
                {"name": "gearbase_hole1", "properties": []},
                {"name": "left_hand", "properties": []},
            ],
            "constraints": [
                {
                    "source": "parallel_box1",
                    "name": "can_manipulate",
                    "target": "shaft1",
                },
                {
                    "source": "shaft1",
                    "name": "can_insert_to",
                    "target": "gearbase_hole1",
                },
            ],
            "relations": [
                {"source": "left_hand", "name": "hold", "target": "parallel_box1"},
                {
                    "source": "shaft1",
                    "name": "is_inserted_to",
                    "target": "gearbase_hole1",
                },
            ],
        }
    ],
}
# --------------------------------------------------------

result = {
    "behavior_tree": {
        "summary": "selector to insert shaft1 into gearbase_hole1",
        "name": "selector: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
        "identifier": 1,
        "type_name": "selector",
        "children": [
            {
                "summary": "condition node to check if shaft1 is inserted to gearbase_hole1",
                "name": "target: is_inserted_to(shaft1, gearbase_hole1)",
                "identifier": 2,
                "type_name": "condition",
                "conditions": [
                    {
                        "object_name": "shaft1",
                        "property_name": "is_inserted_to",
                        "property_value": "gearbase_hole1",
                        "status": True,
                    }
                ],
            },
            {
                "summary": "sequence to insert shaft1 into gearbase_hole1",
                "name": "sequence: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
                "identifier": 3,
                "type_name": "sequence",
                "children": [
                    {
                        "summary": "selector to unload outward_claw",
                        "name": "selector: unload_tool(left_hand, outward_claw)",
                        "identifier": 4,
                        "type_name": "selector",
                        "children": [
                            {
                                "summary": "condition node to check if left_hand is free",
                                "name": "target: is_free(left_hand)",
                                "identifier": 5,
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
                                "summary": "sequence to unload outward_claw",
                                "name": "sequence: unload_tool(left_hand, outward_claw)",
                                "identifier": 6,
                                "type_name": "sequence",
                                "children": [
                                    {
                                        "summary": "condition node to check if left hand holds outward_claw",
                                        "name": "precondition: hold(left_hand, outward_claw)",
                                        "identifier": 7,
                                        "type_name": "condition",
                                        "conditions": [
                                            {
                                                "object_name": "left_hand",
                                                "property_name": "hold",
                                                "property_value": "outward_claw",
                                                "status": True,
                                            }
                                        ],
                                    },
                                    {
                                        "summary": "action node to unload outward_claw from left hand",
                                        "name": "action: unload_tool(left_hand, outward_claw)",
                                        "identifier": 8,
                                        "type_name": "action",
                                        "effects": [
                                            {
                                                "object_name": "left_hand",
                                                "property_name": "is_free",
                                                "property_value": None,
                                                "status": True,
                                            },
                                            {
                                                "object_name": "outward_claw",
                                                "property_name": "is_equippable",
                                                "property_value": None,
                                                "status": True,
                                            },
                                            {
                                                "object_name": "left_hand",
                                                "property_name": "hold",
                                                "property_value": "outward_claw",
                                                "status": False,
                                            },
                                        ],
                                    },
                                ],
                            },
                        ],
                    },
                    {
                        "summary": "selector to load parallel_box1",
                        "name": "selector: load_tool(left_hand, parallel_box1)",
                        "identifier": 9,
                        "type_name": "selector",
                        "children": [
                            {
                                "summary": "condition node to check if left_hand holds parallel_box1",
                                "name": "target: hold(left_hand, parallel_box1)",
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
                                "summary": "sequence to load parallel_box1",
                                "name": "sequence: load_tool(left_hand, parallel_box1)",
                                "identifier": 11,
                                "type_name": "sequence",
                                "children": [
                                    {
                                        "summary": "condition node to check if parallel_box1 is equippable",
                                        "name": "precondition: is_equippable(parallel_box1)",
                                        "identifier": 12,
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
                                        "summary": "condition node to check if left hand is free",
                                        "name": "precondition: is_free(left_hand)",
                                        "identifier": 13,
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
                                        "summary": "action node to equip parallel_box1 to left hand",
                                        "name": "action: load_tool(left_hand, parallel_box1)",
                                        "identifier": 14,
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
                    {
                        "summary": "selector to pick up shaft1 with parallel_box1",
                        "name": "selector: pick_up(left_hand, parallel_box1, shaft1)",
                        "identifier": 15,
                        "type_name": "selector",
                        "children": [
                            {
                                "summary": "condition node to check if parallel_box1 holds shaft1",
                                "name": "target: hold(parallel_box1, shaft1)",
                                "identifier": 16,
                                "type_name": "condition",
                                "conditions": [
                                    {
                                        "object_name": "parallel_box1",
                                        "property_name": "hold",
                                        "property_value": "shaft1",
                                        "status": True,
                                    }
                                ],
                            },
                            {
                                "summary": "sequence to pick up shaft1 with parallel_box1",
                                "name": "sequence: pick_up(left_hand, parallel_box1, shaft1)",
                                "identifier": 17,
                                "type_name": "sequence",
                                "children": [
                                    {
                                        "summary": "condition node to check if parallel_box1 is free",
                                        "name": "precondition: is_free(parallel_box1)",
                                        "identifier": 18,
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
                                        "summary": "condition node to check if left hand holds parallel_box1",
                                        "name": "precondition: hold(left_hand, parallel_box1)",
                                        "identifier": 19,
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
                                        "summary": "action node to pick up shaft1 with parallel_box1",
                                        "name": "action: pick_up(left_hand, parallel_box1, shaft1)",
                                        "identifier": 20,
                                        "type_name": "action",
                                        "effects": [
                                            {
                                                "object_name": "parallel_box1",
                                                "property_name": "hold",
                                                "property_value": "shaft1",
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
                    {
                        "summary": "condition node to check if shaft1 can be inserted to gearbase_hole1",
                        "name": "precondition: can_insert_to(shaft1, gearbase_hole1)",
                        "identifier": 21,
                        "type_name": "condition",
                        "conditions": [
                            {
                                "object_name": "shaft1",
                                "property_name": "can_insert_to",
                                "property_value": "gearbase_hole1",
                                "status": True,
                            }
                        ],
                    },
                    {
                        "summary": "action node to insert shaft1 into gearbase_hole1",
                        "name": "action: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
                        "identifier": 22,
                        "type_name": "action",
                        "effects": [
                            {
                                "object_name": "parallel_box1",
                                "property_name": "hold",
                                "property_value": "shaft1",
                                "status": False,
                            },
                            {
                                "object_name": "parallel_box1",
                                "property_name": "is_free",
                                "property_value": None,
                                "status": True,
                            },
                            {
                                "object_name": "shaft1",
                                "property_name": "is_inserted_to",
                                "property_value": "gearbase_hole1",
                                "status": True,
                            },
                        ],
                    },
                ],
            },
        ],
    },
    "world_state": [
        {
            "objects": [
                {"name": "parallel_box1", "properties": ["is_free", "is_equippable"]},
                {"name": "parallel_box2", "properties": ["is_free", "is_equippable"]},
                {"name": "inward_claw", "properties": ["is_free", "is_equippable"]},
                {"name": "outward_claw", "properties": ["is_free"]},
                {"name": "no_tool", "properties": ["is_free", "is_equippable"]},
                {"name": "gear1", "properties": []},
                {"name": "gear2", "properties": []},
                {"name": "gear3", "properties": []},
                {"name": "shaft1", "properties": []},
                {"name": "shaft2", "properties": []},
                {"name": "shaft3", "properties": []},
                {"name": "gearbase", "properties": []},
                {"name": "gearbase_hole1", "properties": []},
                {"name": "gearbase_hole3", "properties": []},
                {"name": "left_hand", "properties": []},
            ],
            "constraints": [
                {
                    "source": "parallel_box1",
                    "name": "can_manipulate",
                    "target": "shaft1",
                },
                {
                    "source": "parallel_box2",
                    "name": "can_manipulate",
                    "target": "gear1",
                },
                {"source": "outward_claw", "name": "can_manipulate", "target": "gear2"},
                {"source": "outward_claw", "name": "can_manipulate", "target": "gear3"},
                {"source": "no_tool", "name": "can_manipulate", "target": "shaft3"},
                {
                    "source": "shaft1",
                    "name": "can_insert_to",
                    "target": "gearbase_hole1",
                },
                {
                    "source": "shaft3",
                    "name": "can_insert_to",
                    "target": "gearbase_hole3",
                },
                {"source": "gear3", "name": "can_insert_to", "target": "shaft3"},
            ],
            "relations": [
                {"source": "left_hand", "name": "hold", "target": "outward_claw"}
            ],
        }
    ],
}

import json
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree

example1 = {
    "summary": "selector to load left_hand with parallel_box1",
    "name": "selector: load_tool(left_hand, parallel_box1)",
    "children": [
        {
            "summary": "condition node to check if left_hand holds parallel_box1",
            "name": "target: hold(left_hand, parallel_box1)",
        },
        {
            "summary": "sequence to load left_hand with parallel_box1",
            "name": "sequence: load_tool(left_hand, parallel_box1)",
            "children": [
                {
                    "summary": "selector to make left_hand unload outward_claw",
                    "name": "selector: unload_tool(left_hand, outward_claw)",
                    "children": [
                        {
                            "summary": "condition node to check if left_hand is free",
                            "name": "target: is_free(left_hand)",
                        },
                        {
                            "summary": "sequence to make left_hand unload outward_claw",
                            "name": "sequence: unload_tool(left_hand, outward_claw)",
                            "children": [
                                {
                                    "summary": "condition node to check if outward_claw is free",
                                    "name": "precondition: is_free(outward_claw)",
                                },
                                {
                                    "summary": "condition node to check if left hand holds outward_claw",
                                    "name": "precondition: hold(left_hand, outward_claw)",
                                },
                                {
                                    "summary": "action node to make left hand unload outward_claw",
                                    "name": "action: unload_tool(left_hand, outward_claw)",
                                    "effects": [
                                        {
                                            "summary": "left_hand will be free",
                                        },
                                        {
                                            "summary": "outward_claw will be equippable",
                                        },
                                        {
                                            "summary": "left_hand will not hold outward_claw",
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "condition node to check if parallel_box1 is equippable",
                    "name": "precondition: is_equippable(parallel_box1)",
                },
                {
                    "summary": "action node to equip left hand with parallel_box1",
                    "name": "action: load_tool(left_hand, parallel_box1)",
                    "effects": [
                        {
                            "summary": "left_hand will be not free",
                        },
                        {
                            "summary": "parallel_box1 will be not equippable",
                        },
                        {
                            "summary": "left_hand will hold parallel_box1",
                        },
                    ],
                },
            ],
        },
    ],
}

result3 = {
    "summary": "selector to insert shaft1 into gearbase_hole1",
    "name": "selector: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
    "children": [
        {
            "summary": "condition node to check if shaft1 is inserted into gearbase_hole1",
            "name": "target: is_inserted_to(shaft1, gearbase_hole1)",
        },
        {
            "summary": "sequence to insert shaft1 into gearbase_hole1",
            "name": "sequence: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
            "children": [
                {
                    "summary": "selector to unload outward_claw from left_hand",
                    "name": "selector: unload_tool(left_hand, outward_claw)",
                    "children": [
                        {
                            "summary": "condition node to check if left_hand is free",
                            "name": "target: is_free(left_hand)",
                        },
                        {
                            "summary": "sequence to unload outward_claw from left_hand",
                            "name": "sequence: unload_tool(left_hand, outward_claw)",
                            "children": [
                                {
                                    "summary": "condition node to check if left hand holds outward_claw",
                                    "name": "precondition: hold(left_hand, outward_claw)",
                                },
                                {
                                    "summary": "action node to unload outward_claw from left hand",
                                    "name": "action: unload_tool(left_hand, outward_claw)",
                                    "effects": [
                                        {
                                            "summary": "left_hand will be free",
                                            "name": "effect: is_free(left_hand)",
                                        },
                                        {
                                            "summary": "outward_claw will be equippable",
                                            "name": "effect: is_equippable(outward_claw)",
                                        },
                                        {
                                            "summary": "left_hand will not hold outward_claw",
                                            "name": "effect: not hold(left_hand, outward_claw)",
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "selector to load parallel_box1 on left_hand",
                    "name": "selector: load_tool(left_hand, parallel_box1)",
                    "children": [
                        {
                            "summary": "condition node to check if left_hand holds parallel_box1",
                            "name": "target: hold(left_hand, parallel_box1)",
                        },
                        {
                            "summary": "sequence to load parallel_box1 on left_hand",
                            "name": "sequence: load_tool(left_hand, parallel_box1)",
                            "children": [
                                {
                                    "summary": "condition node to check if parallel_box1 is equippable",
                                    "name": "precondition: is_equippable(parallel_box1)",
                                },
                                {
                                    "summary": "condition node to check if left hand is free",
                                    "name": "precondition: is_free(left_hand)",
                                },
                                {
                                    "summary": "action node to load parallel_box1 on left hand",
                                    "name": "action: load_tool(left_hand, parallel_box1)",
                                    "effects": [
                                        {
                                            "summary": "left_hand will not be free",
                                            "name": "effect: not is_free(left_hand)",
                                        },
                                        {
                                            "summary": "parallel_box1 will not be equippable",
                                            "name": "effect: not is_equippable(parallel_box1)",
                                        },
                                        {
                                            "summary": "left_hand will hold parallel_box1",
                                            "name": "effect: hold(left_hand, parallel_box1)",
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "condition node to check if parallel_box1 can manipulate shaft1",
                    "name": "precondition: can_manipulate(parallel_box1, shaft1)",
                },
                {
                    "summary": "condition node to check if shaft1 can be inserted to gearbase_hole1",
                    "name": "precondition: can_insert_to(shaft1, gearbase_hole1)",
                },
                {
                    "summary": "action node to insert shaft1 into gearbase_hole1",
                    "name": "action: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
                    "effects": [
                        {
                            "summary": "parallel_box1 will not hold shaft1",
                            "name": "effect: not hold(parallel_box1, shaft1)",
                        },
                        {
                            "summary": "parallel_box1 will be free",
                            "name": "effect: is_free(parallel_box1)",
                        },
                        {
                            "summary": "shaft1 will be inserted into gearbase_hole1",
                            "name": "effect: is_inserted_to(shaft1, gearbase_hole1)",
                        },
                    ],
                },
            ],
        },
    ],
}

example2 = {
    "summary": "selector to insert gear1 into shaft1",
    "name": "selector: insert(left_hand, parallel_box1, gear1, shaft1)",
    "children": [
        {
            "summary": "condition node to check if gear1 is inserted to shaft1",
            "name": "target: is_inserted_to(gear1, shaft1)",
        },
        {
            "summary": "sequence to insert gear1 into shaft1",
            "name": "sequence: insert(left_hand, parallel_box1, gear1, shaft1)",
            "children": [
                {
                    "summary": "selector to load parallel_box1",
                    "name": "selector: load_tool(left_hand, parallel_box1)",
                    "children": [
                        {
                            "summary": "condition node to check if left_hand holds parallel_box1",
                            "name": "target: hold(left_hand, parallel_box1)",
                        },
                        {
                            "summary": "sequence to load parallel_box1",
                            "name": "sequence: load_tool(left_hand, parallel_box1)",
                            "children": [
                                {
                                    "summary": "condition node to check if parallel_box1 is equippable",
                                    "name": "precondition: is_equippable(parallel_box1)",
                                },
                                {
                                    "summary": "selector to make left_hand unload outward_claw",
                                    "name": "selector: unload_tool(left_hand, outward_claw)",
                                    "children": [
                                        {
                                            "summary": "condition node to check if left_hand is free",
                                            "name": "target: is_free(left_hand)",
                                        },
                                        {
                                            "summary": "sequence to make left_hand unload outward_claw",
                                            "name": "sequence: unload_tool(left_hand, outward_claw)",
                                            "children": [
                                                {
                                                    "summary": "condition node to check if outward_claw is free",
                                                    "name": "precondition: is_free(outward_claw)",
                                                },
                                                {
                                                    "summary": "condition node to check if left hand holds outward_claw",
                                                    "name": "precondition: hold(left_hand, outward_claw)",
                                                },
                                                {
                                                    "summary": "action node to make left hand unload outward_claw",
                                                    "name": "action: unload_tool(left_hand, outward_claw)",
                                                    "effects": [
                                                        {
                                                            "summary": "left_hand will be free",
                                                        },
                                                        {
                                                            "summary": "outward_claw will be equippable",
                                                        },
                                                        {
                                                            "summary": "left_hand will not hold outward_claw",
                                                        },
                                                    ],
                                                },
                                            ],
                                        },
                                    ],
                                },
                                {
                                    "summary": "action node to equip parallel_box1 to left hand",
                                    "name": "action: load_tool(left_hand, parallel_box1)",
                                    "effects": [
                                        {
                                            "summary": "left_hand will be not free",
                                        },
                                        {
                                            "summary": "parallel_box1 will be not equippable",
                                        },
                                        {
                                            "summary": "left_hand will hold parallel_box1",
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "selector to use left_hand with parallel_box1 to pick up gear1",
                    "name": "selector: pick_up(left_hand, parallel_box1, gear1)",
                    "children": [
                        {
                            "summary": "condition node to check if parallel_box1 holds gear1",
                            "name": "target: hold(parallel_box1, gear1)",
                        },
                        {
                            "summary": "sequence to use left hand with parallel_box1 to pick up gear1",
                            "name": "sequence: pick_up(left_hand, parallel_box1, gear1)",
                            "children": [
                                {
                                    "summary": "condition node to check if parallel_box1 is free",
                                    "name": "precondition: is_free(parallel_box1)",
                                },
                                {
                                    "summary": "condition node to check if left hand holds parallel_box1",
                                    "name": "precondition: hold(left_hand, parallel_box1)",
                                },
                                {
                                    "summary": "action node to use left hand with parallel_box1 to pick up gear1",
                                    "name": "action: pick_up(left_hand, parallel_box1, gear1)",
                                    "effects": [
                                        {
                                            "summary": "parallel_box1 will hold gear1",
                                        },
                                        {
                                            "summary": "parallel_box1 will be not free",
                                        },
                                    ],
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "condition node to check if gear1 can be inserted to shaft1",
                    "name": "precondition: can_insert_to(gear1, shaft1)",
                },
                {
                    "summary": "action node to use left_hand with parallel_box1 to insert gear1 to shaft1",
                    "name": "action: insert(left_hand, parallel_box1, gear1, shaft1)",
                    "effects": [
                        {
                            "summary": "parallel_box1 will be not holding gear1",
                        },
                        {
                            "summary": "parallel_box1 will be free",
                        },
                        {
                            "summary": "gear1 will be inserted to shaft1",
                        },
                    ],
                },
            ],
        },
    ],
}

result = {
    "summary": "Selector to insert shaft1 into gearbase_hole1 using parallel_box1 in the left hand",
    "name": "selector: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
    "children": [
        {
            "summary": "Check the target that shaft1 is inserted into gearbase_hole1",
            "name": "target: is_inserted_to(shaft1, gearbase_hole1)",
        },
        {
            "summary": "Sequence to insert shaft1 into gearbase_hole1 using parallel_box1 in the left hand",
            "name": "sequence: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
            "children": [
                {
                    "summary": "Selector to load the parallel_box1 in the left hand",
                    "name": "selector: load_tool(left_hand, parallel_box1)",
                    "children": [
                        {
                            "summary": "Check the target that the left hand is free",
                            "name": "target: is_free(left_hand)",
                        },
                        {
                            "summary": "Sequence to load the tool in the left hand",
                            "name": "sequence: load_tool(left_hand, parallel_box1)",
                            "children": [
                                {
                                    "summary": "Check the precondition that the parallel_box1 is equippable",
                                    "name": "precondition: is_equippable(parallel_box1)",
                                },
                                {
                                    "summary": "Selector to unload the outward_claw in the left hand",
                                    "name": "selector: unload_tool(left_hand, outward_claw)",
                                    "children": [
                                        {
                                            "summary": "Check the target that the left hand is free",
                                            "name": "target: is_free(left_hand)",
                                        },
                                        {
                                            "summary": "Sequence to unload the tool in the left hand",
                                            "name": "sequence: unload_tool(left_hand, outward_claw)",
                                            "children": [
                                                {
                                                    "summary": "Check the precondition that the left hand is holding a outward_claw",
                                                    "name": "precondition: hold(left_hand, outward_claw)",
                                                },
                                                {
                                                    "summary": "Unload the outward_claw in the left hand",
                                                    "name": "action: unload_tool(left_hand, outward_claw)",
                                                },
                                            ],
                                        },
                                    ],
                                },
                                {
                                    "summary": "Load the parallel_box1 in the left hand",
                                    "name": "action: load_tool(left_hand, parallel_box1)",
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "Selector to pick up the shaft1 with the parallel_box1 in the left hand",
                    "name": "selector: pick_up(left_hand, parallel_box1, shaft1)",
                    "children": [
                        {
                            "summary": "Check the target that the parallel_box1 is holding the shaft1",
                            "name": "target: hold(parallel_box1, shaft1)",
                        },
                        {
                            "summary": "Sequence to pick up the shaft1 with the parallel_box1 in the left hand",
                            "name": "sequence: pick_up(left_hand, parallel_box1, shaft1)",
                            "children": [
                                {
                                    "summary": "Check the precondition that the left hand is holding the parallel_box1",
                                    "name": "precondition: hold(left_hand, parallel_box1)",
                                },
                                {
                                    "summary": "Pick up the shaft1 with the parallel_box1 in the left hand",
                                    "name": "action: pick_up(left_hand, parallel_box1, shaft1)",
                                },
                            ],
                        },
                    ],
                },
                {
                    "summary": "Insert shaft1 into gearbase_hole1 using parallel_box1",
                    "name": "action: insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
                },
            ],
        },
    ],
}


def test_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)


# test_bt(result_1["behavior_tree"])
# test_bt(example1)
# test_bt(result3)
# test_bt(example2)
test_bt(result)
