{
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
