import json
import os
import json
import os

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
