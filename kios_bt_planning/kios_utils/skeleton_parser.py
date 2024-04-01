import os
import re
from dotenv import load_dotenv
import json
from dataclasses import dataclass

from pprint import pprint

from kios_bt.data_types import ObjectProperty

"""
only skeleton node parser here
"""


load_dotenv()

data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
# print(data_dir)
world_definition_json = os.path.join(data_dir, "world_definition.json")

with open(world_definition_json, "r") as f:
    world_definition = json.load(f)


@dataclass
class ParsedNode:
    """
    class to store the parsed node (condition or action)
    """

    typename: str
    itemname: str
    params: list[str]


def parse_node_name(name: str) -> ParsedNode:
    """
    parse a node name to extract the type, name and parameters
    typename: selector, sequence, condition, precondition, target, action
    itemname: the name of the condition/action
    params: the parameters of the condition/action

    Example:
        a condition: hold(hand1, tool1)
        an action: pick_up(hand1, tool1, target1)
    """
    pattern = r"(\w+): (\w+)\((.*?)\)"
    match = re.match(pattern, name)
    if match:
        typename = match.group(1)
        itemname = match.group(2)
        params = match.group(3).split(", ") if match.group(3) else []
        return ParsedNode(typename, itemname, params)
    else:
        raise ValueError(f'invalid node name "{name}"!!!')


def parse_node_type(name: str) -> str:

    pattern = r"(selector|sequence|condition|precondition|target|action)"
    match = re.search(pattern, name)
    if match:
        return match.group(1)
    else:
        return None


def ground_action(
    action_name: str, params: list[str]
) -> tuple[list[ObjectProperty], list[ObjectProperty]]:
    """
    furthre ground the parsed action node, extract its precondition and effect according to the definition.

    The definitions are in the world_definition.json file in the "data" directory. See dotenv import sentence.

    return:
        - a list of precondition ObjectProperty
        - a list of effect ObjectProperty

    """
    if world_definition["actions"].get(action_name) is None:
        raise ValueError(f"Action {action_name} is not defined in the world definition")
    action_data = world_definition["actions"][action_name]
    template = action_data["template"]
    precondition = action_data["precondition"]
    effect = action_data["effect"]

    # Extract the placeholders from the template (pattern: ?<name>)
    placeholders = re.findall(r"\?\w+", template)

    # Replace the placeholders one by one with the corresponding parameter value
    for i in range(len(placeholders)):
        escaped_placeholder = re.escape(placeholders[i])
        precondition = [
            re.sub(escaped_placeholder, params[i], pre) for pre in precondition
        ]
        effect = [re.sub(escaped_placeholder, params[i], eff) for eff in effect]

    precondition_list: list[ObjectProperty] = []
    effect_list: list[ObjectProperty] = []

    for item in precondition:
        match = re.match(r"\(?(not )?\(?(\w+) (\w+)? ?(\w+)?\)?\)?", item)

        if match:
            # Extract the property name, object name, and property value from the match object
            status, property_name, object_name, property_value = match.groups()

            # Determine the status based on whether the string starts with "(not"
            status = False if status else True

            # Return a dictionary with the extracted values
            precondition_list.append(
                ObjectProperty(
                    object_name=object_name,
                    property_name=property_name,
                    property_value=property_value if property_value else None,
                    status=status,
                )
            )

    for item in effect:
        match = re.match(r"\(?(not )?\(?(\w+) (\w+)? ?(\w+)?\)?\)?", item)

        if match:
            # Extract the property name, object name, and property value from the match object
            status, property_name, object_name, property_value = match.groups()

            # Determine the status based on whether the string starts with "(not"
            status = False if status else True

            # Return a dictionary with the extracted values
            effect_list.append(
                ObjectProperty(
                    object_name=object_name,
                    property_name=property_name,
                    property_value=property_value if property_value else None,
                    status=status,
                )
            )

    return precondition_list, effect_list


# Test the function with the 'unscrew' action and some parameters
# pprint(ground_action("unscrew", ["a1", "b2", "c3", "d4"]))
# print(ground_action("load_tool", ["a1", "b2"]))
# print(ground_action("put_down", ["a1", "b2", "c3"]))
# print(ground_action("place", ["a1", "b2", "c3", "d4"]))
# pprint(ground_action("detach", ["a1", "b2", "c3", "d4"]))
# pprint(ground_action("insert", ["a1", "b2", "c3", "d4"]))
# print(ground_action("pull", ["a1", "b2", "c3", "d4"]))
# print(ground_action("screw", ["a1", "b2", "c3", "d4"]))
# pprint(ground_action("unload_tool", ["a1", "b2"]))
# print(ground_action("pick_up", ["a1", "b2", "c3"]))
# print(ground_action("unload_tool", ["a1", "b2"]))
