import re
import os
from dotenv import load_dotenv
import json

load_dotenv()

# * kios data dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
print(data_dir)
world_definition_json = os.path.join(data_dir, "world_definition.json")

with open(world_definition_json, "r") as f:
    world_definition = json.load(f)

properties = world_definition["properties"]

constraints = world_definition["constraints"]

relations = world_definition["relations"]

# ! now in json file.
# properties = [
#     "is_free",
#     "is_equippable",
# ]

# constraints = [
#     "can_manipulate",
#     "can_screw_to",
#     "can_insert_to",
#     "can_place_to",
# ]

# relations = [
#     "hold",
#     "is_inserted_to",
#     "is_screwed_to",
#     "is_placed_to",
# ]


def parse_problem_objects(problem: str) -> dict[str, list[dict[str, any]]]:
    pass


def parse_problem_init(
    problem: str, object_list: list[str] = None
) -> dict[str, list[dict[str, any]]]:
    """
    parse to get the world state from the problem.
    pass the object list if you want to add all the objects to the world state.
    """
    init_pattern = r"\(:init\s+(.*?)\s+\)"
    element_pattern = r"\((.*?)\)"

    init_match = re.search(init_pattern, problem, re.DOTALL)
    if init_match:
        init_content = init_match.group(1)
        elements = re.findall(element_pattern, init_content)

        parsed_init = []
        for element in elements:
            parts = element.split()
            name = parts[0]
            args = parts[1:]
            parsed_init.append({"name": name, "args": args})
    else:
        raise ValueError("No init found in the problem")

    if object_list is None:
        object_list = []

    world_state: dict[str, list[dict[str, any]]] = {
        "objects": [],
        "constraints": [],
        "relations": [],
    }

    # add all the objects
    for item in object_list:
        world_state["objects"].append({"name": item, "properties": []})

    for item in parsed_init:
        item_name = item.get("name")
        item_type = classify_item(item_name)

        if item_type == "property":
            world_state["objects"].append(
                {"name": item["args"][0], "properties": [item_name]}  # ! list or item?
            )

        elif item_type == "constraint":
            world_state["constraints"].append(
                {
                    "source": item["args"][0],
                    "name": item_name,
                    "target": item["args"][1],
                }
            )
        elif item_type == "relation":
            world_state["relations"].append(
                {
                    "source": item["args"][0],
                    "name": item_name,
                    "target": item["args"][1],
                }
            )
        else:
            pass

    return world_state


def parse_problem(problem: str) -> dict[str, list[dict[str, any]]]:
    # get objects
    object_dict = parse_problem_objects(problem=problem)

    # merge
    object_list: list[str] = []
    for key, value in object_dict.items():
        # type is not used
        object_list.extend(value)

    # get world_state
    world_state = parse_problem_init(problem=problem, object_list=object_list)

    return world_state


def classify_item(item_name: str) -> str:
    if item_name in properties:
        return "property"
    elif item_name in constraints:
        return "constraint"
    elif item_name in relations:
        return "relation"
    else:
        raise ValueError(f"Unknown item name: {item_name}")


# Usage example:
example_problem = """
(define (problem robot_assembly_problem-problem)
     (:domain robot_assembly_problem-domain)
     (:objects
         parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool
         gear1 gear2 gear3 shaft1 shaft2 shaft3 gearbase gearbase_hole1 gearbase_hole3 - part
         left_hand - hand
     )
     (:init (can_manipulate parallel_box2 gear1) (is_free left_hand) (is_free no_tool) (is_equippable no_tool) (can_manipulate outward_claw gear2) (can_manipulate outward_claw gear3) (can_manipulate no_tool shaft3) (can_manipulate parallel_box1 shaft1) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub lampbase) (can_place_to lamp blub) (can_insert_to shaft1 gearbase_hole1) (can_insert_to shaft3 gearbase_hole3) (can_insert_to gear3 shaft3) (can_insert_to gear2 shaft2) (can_insert_to gear1 shaft1))
     (:goal (and ))
    )"""


def parse_problem_objects(problem: str) -> dict[str, list[str]]:
    objects_start = problem.find("(:objects")
    objects_end = problem.find(")", objects_start)
    objects_section = problem[objects_start:objects_end]

    items = {}
    lines = objects_section.split("\n")
    for line in lines:
        if "-" in line:
            item, *other_items = line.split("-")
            item = item.strip()
            other_items = [item.strip() for item in other_items]
            items[other_items[0]] = item.split()

    return items


def test(problem: str):
    # parsed_init = parse_problem_init(problem)
    # print(parsed_init)
    world_state = parse_problem(problem)
    # for key, value in world_state.items():
    #     print(f"{key}:")
    #     for item in value:
    #         print(item)
    from pprint import pprint

    pprint(world_state)

    pprint(parse_problem_objects(problem))


if __name__ == "__main__":
    test(example_problem)
    pass
