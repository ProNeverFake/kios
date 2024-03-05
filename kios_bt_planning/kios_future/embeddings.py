from kios_utils.parsers import match_type


def embed_ut_nl(unit_subtree: dict) -> str:
    """
    embed the unit subtree into the overall tree
    """
    selector_children = unit_subtree["children"]
    target = ""
    # * target
    for node in selector_children:
        if match_type(node)[0] == "target":
            target += node["summary"]

    sequence_children = selector_children[1]["children"]
    preconditions = []
    action = ""
    for node in sequence_children:
        if match_type(node)[0] == "precondition":
            preconditions.append(node["summary"])
        if match_type(node)[0] == "action":
            action = node["summary"]

    embedding = f"if {' and '.join(preconditions)} then {action}, {target}"

    return embedding


def embed_ft_nl(fulltree: dict) -> str:
    pass



