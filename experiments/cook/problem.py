import re

"""
script to parse the initial state from a problem file.
out-of-date. now pddl part is removed and this part should be no longer in use.
"""


def parse_init(problem):
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

        return parsed_init
    else:
        return []


properties = [
    "is_free",
    "is_equippable",
]

constraints = [
    "can_manipulate",
    "can_screw_to",
    "can_insert_to",
    "can_place_to",
]

relations = [
    "hold",
    "is_inserted_to",
    "is_screwed_to",
    "is_placed_to",
]


def classify_item(item_name):
    if item_name in properties:
        return "property"
    elif item_name in constraints:
        return "constraint"
    elif item_name in relations:
        return "relation"
    else:
        raise ValueError(f"Unknown item name: {item_name}")


# Usage example:
problem = """
(define (problem robot_assembly_problem-problem)
     (:domain robot_assembly_problem-domain)
     (:objects
         parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool
         gear1 gear2 gear3 shaft1 shaft2 shaft3 gearbase gearbase_hole1 gearbase_hole3 - part
         left_hand - hand
     )
     (:init (can_manipulate parallel_box2 gear1) (can_manipulate outward_claw gear2) (can_manipulate outward_claw gear3) (can_manipulate no_tool shaft3) (can_manipulate parallel_box1 shaft1) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub lampbase) (can_place_to lamp blub) (can_insert_to shaft1 gearbase_hole1) (can_insert_to shaft3 gearbase_hole3) (can_insert_to gear3 shaft3) (can_insert_to gear2 shaft2) (can_insert_to gear1 shaft1))
     (:goal (and ))
    )"""

parsed_init = parse_init(problem)

for item in parsed_init:
    item_name = item.get("name")
    item_type = classify_item(item_name)

    world_state = {
        "objects": [],
        "constraints": [],
        "relations": [],
    }

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


print(parsed_init)
