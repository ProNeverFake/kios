import socket
from pprint import pprint
from runtime_script import *
from plan_parser import parse_plan, PlanStep


HOST = "10.157.175.108"  # The server's hostname or IP address
PORT = 65431  # The port used by the server

connected = False
# while not connected:
#     try:
#         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#             s.connect((HOST, PORT))
#             data = s.recv(4096)
#             connected = True
#             if connected:
#                 print("Connected to the server")
#                 s.close()

#     except socket.error as e:
#         print(f"Connection failed: {e}")
# Add any necessary delay before retrying the connection


# print(f"Received {data!r}")


order_dict: dict[str, object] = {
    "outerring": insert_outerring,
    "cone": insert_cone,
    "outputshaftandgearstage2": insert_outputshaftandgearstage2,
    "ringgear": insert_ringgear,
    "designring": insert_designring,
    "gearstage1": insert_gearstage1,
    "driveflange": insert_driveflange,
    "inputpinion": insert_inputpinion,
}

# workflow
# 1. receive plan

# 2. parse plan
# task_plan = parse_plan(data.decode())
task_plan = parse_plan(
    """
0.0000: (initialize defaultgripper) [d:10.0000; c:1.0000]
0.0003: (change_gripper defaultgripper outwardgripper) [d:10.0000; c:1.0000]
10.0005: (initialize outwardgripper) [d:10.0000; c:1.0000]
20.0007: (insert outwardgripper outerring housing) [d:23.0000; c:1.0000]
43.0010: (insert outwardgripper cone outerring) [d:19.0000; c:1.0000]
62.0013: (change_gripper outwardgripper defaultgripper) [d:10.0000; c:1.0000]
72.0015: (initialize defaultgripper) [d:1.0000; c:1.0000]
73.0017: (insert defaultgripper outputshaftandgearstage2 cone) [d:40.0000; c:1.0000]
113.0020: (change_gripper defaultgripper parallelgripper) [d:10.0000; c:1.0000]
123.0023: (initialize parallelgripper) [d:10.0000; c:1.0000]
133.0025: (insert parallelgripper ringgear outputshaftandgearstage2) [d:25.0000; c:1.0000]
158.0027: (change_gripper parallelgripper inwardgripper) [d:25.0000; c:1.0000]
183.0030: (initialize inwardgripper) [d:10.0000; c:1.0000]
193.0033: (insert inwardgripper gearstage1 ringgear) [d:25.0000; c:1.0000]
218.0035: (change_gripper inwardgripper clampgripper) [d:30.0000; c:1.0000]
248.0038: (initialize clampgripper) [d:10.0000; c:1.0000]
258.0040: (insert clampgripper designring ringgear) [d:13.0000; c:1.0000]
271.0042: (change_gripper clampgripper defaultgripper) [d:15.0000; c:1.0000]
286.0045: (initialize defaultgripper) [d:1.0000; c:1.0000]
287.0048: (insert defaultgripper driveflange gearstage1) [d:25.0000; c:1.0000]
312.0050: (insert defaultgripper inputpinion driveflange) [d:25.0000; c:1.0000]
"""
)
# 3. execute plan
set_tool("defaultgripper")


def result_montage(plan_step: PlanStep, execution_time: float, last_result: str) -> str:
    print(type(last_result))
    last_result += f"{plan_step.raw_action}, {execution_time}\n"
    return last_result


result = ""
for plan_step in task_plan:
    pprint(f"Executing {plan_step}")
    action_name = plan_step.action_name
    action_params = plan_step.action_params
    execution_time = 0
    if action_name == "change_gripper":
        execution_time = change_gripper(action_params[0], action_params[1])
    elif action_name == "initialize":
        execution_time = initialize_gripper()
    elif action_name == "insert":
        part_name = action_params[1]
        execution_time = order_dict[part_name]()
    else:
        raise ValueError(f"Unknown action name: {action_name}")

    result = result_montage(plan_step, execution_time, result)

pprint(result)
file_path = os.path.join(os.path.dirname(__file__), "result.txt")
with open(file_path, "w") as f:
    f.write(result)
    f.close()

# 4. send execution time back


def test_montage():
    result = ""
    for plan_step in task_plan:
        result = result_montage(plan_step, 1, result)

    pprint(result)


if __name__ == "__main__":
    # test_montage()
    pass
