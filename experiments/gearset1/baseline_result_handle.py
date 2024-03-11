import json
import os
import json
import os


def remove_entities():
    file_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.join(file_dir, "baseline_result.jsonl")

    with open(result_dir, "r") as file:
        results = file.read()
        data = [json.loads(line) for line in results.splitlines()]

    for item in data:
        ws = item["initial_world_state"]
        objs: list[dict] = ws["objects"]
        cstrs = ws["constraints"]
        for obj in objs:
            if obj["name"] in ["parallel_box1", "parallel_box2", "outward_claw", "inward_claw", "no_tool"]:
                print(f"Removing {obj['name']}")
                objs.remove(obj)
        
        for cstr in cstrs:
            if cstr["source"] in ["parallel_box1", "parallel_box2", "outward_claw", "inward_claw", "no_tool"]:
                cstrs.remove(cstr)
            elif cstr["target"] in ["parallel_box1", "parallel_box2", "outward_claw", "inward_claw", "no_tool"]:
                cstrs.remove(cstr)
            else:
                pass

    with open(result_dir, "w") as file:
        for item in data:
            file.write(json.dumps(item))
            file.write("\n")



def handle_baseline_results():
    file_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.join(file_dir, "baseline_result.jsonl")

    if not os.path.exists(os.path.join(file_dir, "problem_set")):
        os.makedirs(os.path.join(file_dir, "problem_set"))

    with open(result_dir, "r") as file:
        results = file.read()
        data = [json.loads(line) for line in results.splitlines()]

    i = 0
    for item in data:
        i_str = str(i).zfill(3)
        entity_path = os.path.join(file_dir, "problem_set", f"problem_{i_str}.json")
        with open(entity_path, "w") as file:
            file.write(json.dumps(item, indent=4))
        i += 1


if __name__ == "__main__":
    handle_baseline_results()
    # remove_entities()
    pass
