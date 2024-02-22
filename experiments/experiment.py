
# * generate a dict

action_dict : dict{str, object}


action = "insert(out_ring, housing)"

def parse_action(action):
    action = action.split("(")
    action[1] = action[1].split(")")[0]
    return part_name

part_name = parse_action(action)

robot_action = action_dict.get(part_name)

execute(action)
