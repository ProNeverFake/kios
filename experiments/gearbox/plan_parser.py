"""
not BB-written
"""


class PlanStep:
    def __init__(
        self,
        start_time: float,
        estimated_duration: float,
        action_name: str,
        action_params: list[str],
    ):
        self.start_time = start_time
        self.estimated_duration = estimated_duration
        self.estimated_end_time = start_time + estimated_duration
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

        # Extract the action name and parameters
        action_details = actions.strip("()").split(" ")  # Remove parentheses and split
        action_name = action_details[0]
        action_params = action_details[1:]

        # Create a PlanStep instance for each line and append it to the plan_steps list
        plan_step = PlanStep(start_time, duration, action_name, action_params)
        plan_steps.append(plan_step)

    return plan_steps


# Example usage with your provided plan
plan_str = """0.000: (change_gripper defaulgripper outwardgripper)  [10.000]
10.001: (insert outwardgripper outerring housing)  [23.000]
33.002: (insert outwardgripper cone outerring)  [19.000]
52.003: (change_gripper outwardgripper defaulgripper)  [10.000]
62.004: (insert defaulgripper outputshaftandgearstage2 cone)  [40.000]
102.005: (insert defaulgripper ringgear outputshaftandgearstage2)  [12.000]
114.006: (insert defaulgripper designring ringgear)  [10.000]
124.007: (change_gripper defaulgripper parallelgripper)  [10.000]
134.008: (insert parallelgripper gearstage1 ringgear)  [25.000]
159.009: (change_gripper parallelgripper outwardgripper)  [20.000]
179.010: (insert outwardgripper driveflange ringgear)  [25.000]
204.011: (change_gripper outwardgripper clampgripper)  [25.000]
229.012: (insert clampgripper inputpinion driveflange)  [13.000]"""

'''
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

'''

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
for step in parsed_plan:
    print(step)
