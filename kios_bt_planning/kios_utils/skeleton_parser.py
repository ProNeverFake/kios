import os
import re

# from dotenv import load_dotenv
import json


def parse_node_name(name: str) -> dict:
    pattern = r"(\w+): (\w+)\((.*?)\)"
    match = re.match(pattern, name)
    if match:
        typename = match.group(1)
        itemname = match.group(2)
        params = match.group(3).split(", ") if match.group(3) else []
        return {"typename": typename, "itemname": itemname, "params": params}
    else:
        raise ValueError("Invalid node name!!!")
