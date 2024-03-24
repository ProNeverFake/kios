"""
not BB-written
"""

from pprint import pprint
import re


class PlanStep:
    def __init__(
        self,
        start_time: float,
        estimated_duration: float,
        action_name: str,
        action_params: list[str],
        raw_action: str,
    ):
        self.start_time = start_time
        self.estimated_duration = estimated_duration
        self.estimated_end_time = start_time + estimated_duration
        self.action_name = action_name
        self.action_params = action_params
        self.raw_action = raw_action

    def __repr__(self):
        return f"PlanStep({self.action_name}, {self.action_params}, raw is {self.raw_action})"


def parse_plan(plan_str: str) -> list[PlanStep]:
    plan_steps = []
    # Split the input string into lines
    lines = plan_str.strip().split("\n")
    for line in lines:
        matches = re.findall(r"\((.*?)\)", line)
        # pprint(matches)
        action_details = matches[0].split(" ")
        action_name = action_details[0]
        action_params = action_details[1:]

        # Create a PlanStep instance for each line and append it to the plan_steps list
        plan_step = PlanStep(0, 0, action_name, action_params, raw_action=line)
        plan_steps.append(plan_step)

    return plan_steps


# Example usage with your provided plan
plan_str = """0.0003: (change_gripper defaulgripper outwardgripper) [d:10.0000; c:1.0000]\n10.0005: (initialize outwardgripper) [d:10.0000; c:1.0000]\n20.0007: (insert outwardgripper outerring housing) [d:23.0000; c:1.0000]\n43.0010: (insert outwardgripper cone outerring) [d:19.0000; c:1.0000]\n62.0013: (change_gripper outwardgripper defaulgripper) [d:10.0000; c:1.0000]\n72.0015: (initialize defaulgripper) [d:1.0000; c:1.0000]\n73.0017: (insert defaulgripper outputshaftandgearstage2 cone) [d:40.0000; c:1.0000]\n113.0020: (change_gripper defaulgripper parallelgripper) [d:10.0000; c:1.0000]\n123.0023: (initialize parallelgripper) [d:10.0000; c:1.0000]\n133.0025: (insert parallelgripper ringgear outputshaftandgearstage2) [d:25.0000; c:1.0000]\n158.0027: (change_gripper parallelgripper inwardgripper) [d:25.0000; c:1.0000]\n183.0030: (initialize inwardgripper) [d:10.0000; c:1.0000]\n193.0033: (insert inwardgripper gearstage1 ringgear) [d:25.0000; c:1.0000]\n218.0035: (change_gripper inwardgripper clampgripper) [d:30.0000; c:1.0000]\n248.0038: (initialize clampgripper) [d:10.0000; c:1.0000]\n258.0040: (insert clampgripper designring ringgear) [d:13.0000; c:1.0000]\n271.0042: (change_gripper clampgripper defaulgripper) [d:15.0000; c:1.0000]\n286.0045: (initialize defaulgripper) [d:1.0000; c:1.0000]\n287.0048: (insert defaulgripper driveflange gearstage1) [d:25.0000; c:1.0000]\n312.0050: (insert defaulgripper inputpinion driveflange) [d:25.0000; c:1.0000]"""

# """0.000: (change_gripper defaulgripper outwardgripper)  [10.000]
# 10.001: (insert outwardgripper outerring housing)  [23.000]
# 33.002: (insert outwardgripper cone outerring)  [19.000]
# 52.003: (change_gripper outwardgripper defaulgripper)  [10.000]
# 62.004: (insert defaulgripper outputshaftandgearstage2 cone)  [40.000]
# 102.005: (insert defaulgripper ringgear outputshaftandgearstage2)  [12.000]
# 114.006: (insert defaulgripper designring ringgear)  [10.000]
# 124.007: (change_gripper defaulgripper parallelgripper)  [10.000]
# 134.008: (insert parallelgripper gearstage1 ringgear)  [25.000]
# 159.009: (change_gripper parallelgripper outwardgripper)  [20.000]
# 179.010: (insert outwardgripper driveflange ringgear)  [25.000]
# 204.011: (change_gripper outwardgripper clampgripper)  [25.000]
# 229.012: (insert clampgripper inputpinion driveflange)  [13.000]"""

"""
object list
defaultgripper
outwardgripper
outerring housingringhole
cone outerring
defaultgripper
outputshaftandgearstage2 housinginternalgear
ringgear outputshaftandgearstage2 #!
designring ringgear #?
parallelgripper
gearstage1 ringgear #!
driveflange ringgear #?
clampgripper
inputpinion driveflange #!

"""

"""
change_gripper defaulgripper outwardgripper
insert outwardgripper outerring [housing_ringhole]
insert outwardgripper1 cone1 outerring1 #? 
change_gripper outwardgripper1 defaulgripper1
insert defaulgripper1 outputshaftandgearstage21 housing_internalgear1
insert defaulgripper1 ringgear1 outputshaftandgearstage21 #?
insert defaulgripper1 designring1 ringgear1 #?
change_gripper defaulgripper1 parallelgripper1
insert parallelgripper1 gearstage11 ringgear1 #?
change_gripper parallelgripper1 outwardgripper1
insert outwardgripper1 driveflange1 gearstage11 #!
change_gripper outwardgripper1 clampgripper1
insert clampgripper1 inputpinion1 driveflange1 #?   
"""

parsed_plan = parse_plan(plan_str)

# Display the parsed plan steps
# for step in parsed_plan:
#     pprint(step)
# pprint(parsed_plan)
