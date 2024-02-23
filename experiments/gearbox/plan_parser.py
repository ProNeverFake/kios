"""
not BB-written
"""


class PlanStep:
    def __init__(
        self,
        start_time: float,
        end_time: float,
        action_name: str,
        action_params: list[str],
    ):
        self.start_time = start_time
        self.end_time = end_time
        self.action_name = action_name
        self.action_params = action_params

    def __repr__(self):
        return f"PlanStep(start_time={self.start_time}, end_time={self.end_time}, action_name='{self.action_name}', action_params={self.action_params})"


def parse_plan(plan_str: str) -> list[PlanStep]:
    plan_steps = []
    # Split the input string into lines
    lines = plan_str.strip().split("\n")
    for line in lines:
        # Extract the start time, action details, and duration from each line
        start_time_str, actions_and_duration = line.split(": ")
        start_time = float(start_time_str)
        actions, duration_str = actions_and_duration.split("  [")
        duration = float(duration_str[:-1])  # Remove the closing bracket
        end_time = start_time + duration

        # Extract the action name and parameters
        action_details = actions.strip("()").split(" ")  # Remove parentheses and split
        action_name = action_details[0]
        action_params = action_details[1:]

        # Create a PlanStep instance for each line and append it to the plan_steps list
        plan_step = PlanStep(start_time, end_time, action_name, action_params)
        plan_steps.append(plan_step)

    return plan_steps


# Example usage with your provided plan
plan_str = """0.000: (change_gripper defaulgripper1 outwardgripper1)  [10.000]
10.001: (insert outwardgripper1 outerring1 housing1)  [23.000]
33.002: (insert outwardgripper1 cone1 outerring1)  [19.000]
52.003: (change_gripper outwardgripper1 defaulgripper1)  [10.000]
62.004: (insert defaulgripper1 outputshaftandgearstage21 cone1)  [40.000]
102.005: (insert defaulgripper1 ringgear1 outputshaftandgearstage21)  [12.000]
114.006: (insert defaulgripper1 designring1 ringgear1)  [10.000]
124.007: (change_gripper defaulgripper1 parallelgripper1)  [10.000]
134.008: (insert parallelgripper1 gearstage11 ringgear1)  [25.000]
159.009: (change_gripper parallelgripper1 outwardgripper1)  [20.000]
179.010: (insert outwardgripper1 driveflange1 gearstage11)  [25.000]
204.011: (change_gripper outwardgripper1 clampgripper1)  [25.000]
229.012: (insert clampgripper1 inputpinion1 driveflange1)  [13.000]"""

"""
change_gripper defaulgripper1 outwardgripper1
insert outwardgripper1 outerring1 [housing_ringhole1]
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
for step in parsed_plan:
    print(step)
