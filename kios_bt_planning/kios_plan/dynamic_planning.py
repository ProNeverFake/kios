hand_list = [
    "left_hand",
]
tool_list = [
    "inwardgripper",
    "outwardgripper",
    "parallelgripper",
    "clampgripper",
    "defaultgripper",
]
part_list = [
    "gear1",
    "shaft1",
    "gear2",
    "shaft2",
    "gear3",
    "shaft3",
    "gearbase_hole1",
    "gearbase_hole3",
    "gearbase",
]


def generate_unit_tree_entry(action_create_func: object, kwargs: dict):
    """
    create action
    """
    return action_create_func(**kwargs)


def create_pick_up(hand: str, tool: str, part1: str):
    """
    create pick up action
    """
    target = f"hold({tool}, {part1})"
    ut = {
        "summary": f"selector to pick up the {part1} with the {tool} in the {hand}",
        "name": f"selector: pick_up({hand}, {tool}, {part1})",
        "children": [
            {
                "summary": f"the target is to make the {tool} hold the {part1}",
                "name": f"target: hold({tool}, {part1})",
            },
            {
                "summary": f"sequence to pick up the {part1} with the {tool} in the {hand}",
                "name": f"sequence: pick_up({hand}, {tool}, {part1})",
                "children": [
                    {
                        "summary": f"a precondition is the {tool} is empty",
                        "name": f"precondition: is_empty({tool})",
                    },
                    {
                        "summary": f"a precondition is the {hand} is holding {tool}",
                        "name": f"precondition: hold({hand}, {tool})",
                    },
                    {
                        "summary": f"the action to pick up the {part1} with the {tool} in the {hand}",
                        "name": f"action: pick_up({hand}, {tool}, {part1})",
                    },
                ],
            },
        ],
    }
    return target, ut


def create_change_tool(hand: str, tool1: str, tool2: str):
    """
    create change tool action
    """
    target = f"hold({hand}, {tool2})"
    ut = {
        "summary": f"selector to change tool in {hand} from {tool1} to {tool2}",
        "name": f"selector: change_tool({hand}, {tool1}, {tool2})",
        "children": [
            {
                "summary": f"the target is to make {hand} hold {tool2}",
                "name": f"target: hold({hand}, {tool2})",
            },
            {
                "summary": f"sequence to change tool in {hand} from {tool1} to {tool2}",
                "name": f"sequence: change_tool({hand}, {tool1}, {tool2})",
                "children": [
                    {
                        "summary": f"a precondition is {hand} is holding {tool1}",
                        "name": f"precondition: hold({hand}, {tool1})",
                    },
                    {
                        "summary": f"a precondition is {tool1} is empty",
                        "name": f"precondition: is_empty({tool1})",
                    },
                    {
                        "summary": f"the action to change tool in {hand} from {tool1} to {tool2}",
                        "name": f"action: change_tool({hand}, {tool1}, {tool2})",
                    },
                ],
            },
        ],
    }
    return target, ut


def create_put_down(hand: str, tool: str, part: str):
    """
    create put down action
    """
    target = f"is_empty({tool})"
    ut = {
        "summary": f"selector to put down the {part} held by the {tool} in the {hand}",
        "name": f"selector: put_down({hand}, {tool}, {part})",
        "children": [
            {
                "summary": f"the target is to make the {tool} empty",
                "name": f"target: is_empty({tool})",
            },
            {
                "summary": f"sequence to put down the {part} held by the {tool} in the {hand}",
                "name": f"sequence: put_down({hand}, {tool}, {part})",
                "children": [
                    {
                        "summary": f"a precondition is the {hand} is holding {tool}",
                        "name": f"precondition: hold({hand}, {tool})",
                    },
                    {
                        "summary": f"a precondition is the {tool} is holding {part}",
                        "name": f"precondition: hold({tool}, {part})",
                    },
                    {
                        "summary": f"the action to put down the {part} held by the {tool} in the {hand}",
                        "name": f"action: put_down({hand}, {tool}, {part})",
                    },
                ],
            },
        ],
    }
    return target, ut


def create_insert(hand: str, tool: str, part1: str, part2: str):
    """
    create insert action
    """

    target = f"is_inserted_to({part1}, {part2})"
    ut = {
        "summary": f"selector to insert the {part1} into the {part2} with the {tool} in the {hand}",
        "name": f"selector: insert({hand}, {tool}, {part1}, {part2})",
        "children": [
            {
                "summary": f"the target is to make the {part1} be inserted into the {part2}",
                "name": f"target: is_inserted_to({part1}, {part2})",
            },
            {
                "summary": f"sequence to insert the {part1} into the {part2} with the {tool} in the {hand}",
                "name": f"sequence: insert({hand}, {tool}, {part1}, {part2})",
                "children": [
                    {
                        "summary": f"a precondition is the {hand} is holding {tool}",
                        "name": f"precondition: hold({hand}, {tool})",
                    },
                    {
                        "summary": f"a precondition is the {tool} is holding {part1}",
                        "name": f"precondition: hold({tool}, {part1})",
                    },
                    {
                        "summary": f"the action to insert the {part1} into the {part2} with the {tool} in the {hand}",
                        "name": f"action: insert({hand}, {tool}, {part1}, {part2})",
                    },
                ],
            },
        ],
    }
    return target, ut


def create_screw(hand: str, tool: str, part1: str, part2: str):
    """
    create screw action
    """
    target = f"is_screwed_to({part1}, {part2})"
    unit_tree = {
        "summary": f"selector to screw the {part1} into the {part2} with the {tool} in the {hand}",
        "name": f"selector: screw({hand}, {tool}, {part1}, {part2})",
        "children": [
            {
                "summary": f"the target is to make the {part1} be screwed into the {part2}",
                "name": f"target: is_screwed_to({part1}, {part2})",
            },
            {
                "summary": f"sequence to screw the {part1} into the {part2} with the {tool} in the {hand}",
                "name": f"sequence: screw({hand}, {tool}, {part1}, {part2})",
                "children": [
                    {
                        "summary": f"a precondition is the {hand} is holding {tool}",
                        "name": f"precondition: hold({hand}, {tool})",
                    },
                    {
                        "summary": f"a precondition is the {tool} is holding {part1}",
                        "name": f"precondition: hold({tool}, {part1})",
                    },
                    {
                        "summary": f"the action to screw the {part1} into the {part2} with the {tool} in the {hand}",
                        "name": f"action: screw({hand}, {tool}, {part1}, {part2})",
                    },
                ],
            },
        ],
    }
    return target, unit_tree


def create_place(hand: str, tool: str, part1: str, part2: str):
    """
    create place action
    """
    target = f"is_placed_to({part1}, {part2})"
    unit_tree = {
        "summary": f"selector to place the {part1} into the {part2} with the {tool} in the {hand}",
        "name": f"selector: place({hand}, {tool}, {part1}, {part2})",
        "children": [
            {
                "summary": f"the target is to make the {part1} be placed into the {part2}",
                "name": f"target: is_placed_to({part1}, {part2})",
            },
            {
                "summary": f"sequence to place the {part1} into the {part2} with the {tool} in the {hand}",
                "name": f"sequence: place({hand}, {tool}, {part1}, {part2})",
                "children": [
                    {
                        "summary": f"a precondition is the {hand} is holding {tool}",
                        "name": f"precondition: hold({hand}, {tool})",
                    },
                    {
                        "summary": f"a precondition is the {tool} is holding {part1}",
                        "name": f"precondition: hold({tool}, {part1})",
                    },
                    {
                        "summary": f"the action to place the {part1} into the {part2} with the {tool} in the {hand}",
                        "name": f"action: place({hand}, {tool}, {part1}, {part2})",
                    },
                ],
            },
        ],
    }
    return target, unit_tree


gearset_ut_dict: dict[str, dict] = {}
lamp_ut_dict: dict[str, dict] = {}
chair_ut_dict: dict[str, dict] = {}
# * change tool
# for tool1 in tool_list:
#     for tool2 in tool_list:
#         if tool1 != tool2:
#             target, unit_tree = create_change_tool("left_hand", tool1, tool2)
#             gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "defaultgripper", "parallelgripper")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "clampgripper", "parallelgripper")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "parallelgripper")
# gearset_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "inwardgripper", "parallelgripper")
gearset_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "clampgripper")
# gearset_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "defaultgripper", "clampgripper")
gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "clampgripper")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "clampgripper")
# gearset_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "clampgripper", "outwardgripper")
# gearset_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "defaultgripper", "outwardgripper")
gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "outwardgripper")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "outwardgripper")
# gearset_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "clampgripper", "defaultgripper")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "defaultgripper")
# gearset_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "parallelgripper", "defaultgripper")
gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "defaultgripper")
# gearset_ut_dict[target] = unit_tree

# * pick up
target, unit_tree = create_pick_up("left_hand", "clampgripper", "shaft1")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "defaultgripper", "shaft3")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "parallelgripper", "gear1")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "outwardgripper", "gear2")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "outwardgripper", "gear3")
gearset_ut_dict[target] = unit_tree

# * put down
# target, unit_tree = create_put_down("left_hand", "clampgripper", "shaft1")
# gearset_ut_dict[target] = unit_tree
target, unit_tree = create_put_down("left_hand", "parallelgripper", "gear1")
gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "outwardgripper", "gear2")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "outwardgripper", "gear3")
# gearset_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "defaultgripper", "shaft3")
# gearset_ut_dict[target] = unit_tree
# * insert
target, unit_tree = create_insert(
    "left_hand", "clampgripper", "shaft1", "gearbase_hole1"
)
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_insert("left_hand", "parallelgripper", "gear1", "shaft1")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_insert("left_hand", "outwardgripper", "gear2", "shaft2")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_insert("left_hand", "outwardgripper", "gear3", "shaft3")
gearset_ut_dict[target] = unit_tree
target, unit_tree = create_insert(
    "left_hand", "defaultgripper", "shaft3", "gearbase_hole3"
)
gearset_ut_dict[target] = unit_tree
# * screw
# ! no action
# * place
# ! no action

#############################################################################* lamp
# * change tool
# for tool1 in tool_list:
#     for tool2 in tool_list:
#         if tool1 != tool2:
#             target, unit_tree = create_change_tool("left_hand", tool1, tool2)
#             lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "defaultgripper", "parallelgripper")
# lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "clampgripper", "parallelgripper")
# lamp_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "outwardgripper", "parallelgripper")
lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "parallelgripper")
# lamp_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "clampgripper")
# lamp_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "defaultgripper", "clampgripper")
lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "clampgripper")
# lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "clampgripper")
# lamp_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "clampgripper", "outwardgripper")
# lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "defaultgripper", "outwardgripper")
# lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "outwardgripper")
# lamp_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "inwardgripper", "outwardgripper")
lamp_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "clampgripper", "defaultgripper")
# lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "defaultgripper")
# lamp_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "parallelgripper", "defaultgripper")
lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "defaultgripper")
# lamp_ut_dict[target] = unit_tree

# * pick up
target, unit_tree = create_pick_up("left_hand", "clampgripper", "lampbulb")
lamp_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "outwardgripper", "lampshade")
lamp_ut_dict[target] = unit_tree


# * put down
# target, unit_tree = create_put_down("left_hand", "clampgripper", "lampbulb")
# lamp_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "outwardgripper", "lampshade")
# lamp_ut_dict[target] = unit_tree
target, unit_tree = create_put_down("left_hand", "defaultgripper", "cube")
lamp_ut_dict[target] = unit_tree


# * actions
target, unit_tree = create_screw("left_hand", "clampgripper", "lampbulb", "lampbase")
lamp_ut_dict[target] = unit_tree
target, unit_tree = create_place("left_hand", "outwardgripper", "lampshade", "lampbulb")
lamp_ut_dict[target] = unit_tree

#############################################################################* chair

# * change tool
# for tool1 in tool_list:
#     for tool2 in tool_list:
#         if tool1 != tool2:
#             target, unit_tree = create_change_tool("left_hand", tool1, tool2)
#             chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "defaultgripper", "parallelgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "clampgripper", "parallelgripper")
# chair_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "outwardgripper", "parallelgripper")
chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "parallelgripper")
# chair_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "clampgripper")
# chair_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "defaultgripper", "clampgripper")
chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "clampgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "clampgripper")
# chair_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "clampgripper", "outwardgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "defaultgripper", "outwardgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "outwardgripper")
# chair_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "inwardgripper", "outwardgripper")
chair_ut_dict[target] = unit_tree

# target, unit_tree = create_change_tool("left_hand", "clampgripper", "defaultgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "defaultgripper")
# chair_ut_dict[target] = unit_tree
target, unit_tree = create_change_tool("left_hand", "parallelgripper", "defaultgripper")
chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "inwardgripper", "defaultgripper")
# chair_ut_dict[target] = unit_tree

target, unit_tree = create_change_tool("left_hand", "clampgripper", "inwardgripper")
chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "outwardgripper", "inwardgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "defaultgripper", "inwardgripper")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_change_tool("left_hand", "parallelgripper", "inwardgripper")
# chair_ut_dict[target] = unit_tree

# * pick up
target, unit_tree = create_pick_up("left_hand", "clampgripper", "chairback")
chair_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "inwardgripper", "chairnut1")
chair_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "inwardgripper", "chairnut2")
chair_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "defaultgripper", "chairleg1")
chair_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "defaultgripper", "chairleg2")
chair_ut_dict[target] = unit_tree
target, unit_tree = create_pick_up("left_hand", "defaultgripper", "chairseat")
chair_ut_dict[target] = unit_tree


# * put down
# target, unit_tree = create_put_down("left_hand", "clampgripper", "chairback")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "inwardgripper", "chairnut1")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "inwardgripper", "chairnut2")
# chair_ut_dict[target] = unit_tree
target, unit_tree = create_put_down("left_hand", "defaultgripper", "chairleg1")
chair_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "defaultgripper", "chairleg2")
# chair_ut_dict[target] = unit_tree
# target, unit_tree = create_put_down("left_hand", "defaultgripper", "chairseat")
# chair_ut_dict[target] = unit_tree


# * actions
target, unit_tree = create_screw(
    "left_hand", "inwardgripper", "chairnut1", "chairseatbolt1"
)
chair_ut_dict[target] = unit_tree
target, unit_tree = create_screw(
    "left_hand", "inwardgripper", "chairnut2", "chairseatbolt2"
)
chair_ut_dict[target] = unit_tree
target, unit_tree = create_insert(
    "left_hand", "clampgripper", "chairback", "chairseatconnector"
)
chair_ut_dict[target] = unit_tree
target, unit_tree = create_screw(
    "left_hand", "defaultgripper", "chairleg1", "chairseatthread1"
)
chair_ut_dict[target] = unit_tree
target, unit_tree = create_screw(
    "left_hand", "defaultgripper", "chairnut2", "chairseatthread2"
)
