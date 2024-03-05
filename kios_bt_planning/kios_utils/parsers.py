import re

def match_type(node: dict) -> tuple[str, str]:
    node_name = node["name"]
    match = re.search(
        r"(selector|sequence|action|precondition|condition|target):\s*(.+)", node_name
    )
    if match:
        node_type = match.group(1)
        node_body = match.group(2)
        return node_type, node_body
    else:
        raise ValueError(f"the node name {node_name} does not match any type.")
