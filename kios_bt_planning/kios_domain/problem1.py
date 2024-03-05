from kios_domain.knowledge_v0 import *

########################## * current problem context ######################
# add all the related instances to the problem here

# * hand instances
problem.add_object(left_hand)

# * tool instances
problem.add_object(parallel_box1)
problem.add_object(parallel_box2)
problem.add_object(inward_claw)
problem.add_object(outward_claw)
problem.add_object(no_tool)

# * part instances
# * chair
# not this time
# * lamp
# not this time
# * gearset
problem.add_object(gear1)
problem.add_object(gear2)
problem.add_object(gear3)
problem.add_object(shaft1)
problem.add_object(shaft2)
problem.add_object(gearset_base)


########################## * initial world state ##########################
# ! BBNOTE: LLM CONSENTRATION FOR STATE KEEPING LOAD SAVING
# ! Default initial value: hand is free, tools are equippable and free, parts are not assembled
problem.set_initial_value(is_free(left_hand), True)
problem.set_initial_value(is_free(parallel_box1), True)
problem.set_initial_value(is_free(parallel_box2), True)
problem.set_initial_value(is_free(inward_claw), True)
problem.set_initial_value(is_free(outward_claw), True)

problem.set_initial_value(is_equippable(parallel_box1), True)
problem.set_initial_value(is_equippable(parallel_box2), True)
problem.set_initial_value(is_equippable(inward_claw), True)
problem.set_initial_value(is_equippable(outward_claw), True)


########################## * goal ##########################################
problem.add_goal(is_inserted_to(shaft1, gearset_base))
# print(problem)

########################## * planning ######################################
# with OneshotPlanner(name="pyperplan") as planner:
#     result = planner.solve(problem)
#     if result.status == up.engines.PlanGenerationResultStatus.SOLVED_SATISFICING:
#         print("Pyperplan returned: %s" % result.plan)
#     else:
#         print("No plan found.")

# with OneshotPlanner(problem_kind=problem.kind) as planner:
#     result = planner.solve(problem)
#     print("%s returned: %s" % (planner.name, result.plan))
