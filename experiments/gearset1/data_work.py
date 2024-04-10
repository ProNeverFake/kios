import os
import json

current_dir = os.path.dirname(__file__)

method_dir_name = "one_step_record"

method_dir = os.path.join(current_dir, method_dir_name)

folder_count = len(
    [
        name
        for name in os.listdir(method_dir)
        if os.path.isdir(os.path.join(method_dir, name))
    ]
)

success_count = 0

for i in range(folder_count):
    folder_path = os.path.join(method_dir, str(i))
    json_file_path = os.path.join(folder_path, "result.json")
    with open(json_file_path, "r") as f:
        result = json.load(f)
    if result["tree_result"]["result"] == "error":
        success_count += 1

print(f"Success rate: {success_count}/{folder_count}")
