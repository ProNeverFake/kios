from unified_planning.shortcuts import *

######################################## Type ########################################
Location = UserType("Location")
Thing = UserType("Thing")
Tool = UserType("Tool", father=Thing)
Part = UserType("Part", father=Thing)

######################################## Fluent ########################################
reachable = unified_planning.model.Fluent(
    "reachable", BoolType(), l_from=Location, l_to=Location
)
robot_at = unified_planning.model.Fluent("robot_at", BoolType(), l=Location)
thing_at = unified_planning.model.Fluent("thing_at", BoolType(), t=Thing, l=Location)

in_tool = Fluent("in_tool", p=Part, tool=Tool)
in_hand = Fluent("in_hand", BoolType(), tool=Tool)

interactive = Fluent("interactive", BoolType(), p=Part, tool=Tool)

inserted = Fluent("inserted", BoolType(), p=Part)
screwed = Fluent("screwed", BoolType(), p=Part)
placed = Fluent("placed", BoolType(), p=Part)

######################################## Action ########################################

# * move
move = InstantaneousAction("move", l_from=Location, l_to=Location)

l_from = move.parameter("l_from")
l_to = move.parameter("l_to")

move.add_precondition(robot_at(l_from))
move.add_precondition(reachable(l_from, l_to))

move.add_effect(robot_at(l_from), False)
move.add_effect(robot_at(l_to), True)

# * load_tool
load_tool = unified_planning.model.InstantaneousAction(
    "load_tool",
    tool=Tool,
)
tool = load_tool.parameter("tool")

tool_var = Variable("tool_var", Tool)
load_tool.add_precondition(Not(Exists(in_hand(tool_var)), tool_var))

load_tool.add_effect(in_hand(tool), True)

# * unload_tool
unload_tool = unified_planning.model.InstantaneousAction("unload_tool", tool=Tool)
tool = unload_tool.parameter("tool")

unload_tool.add_precondition(in_hand(tool))
p_var = Variable("p_var", Part)
unload_tool.add_precondition(Not(Exists(in_tool(p_var, tool), p_var)))

unload_tool.add_effect(in_hand(tool), False)

# * pick
pick = unified_planning.model.InstantaneousAction(
    "pick", p=Part, tool=Tool, l_from=Location, l_to=Location
)
p = pick.parameter("p")
tool = pick.parameter("tool")
l_from = pick.parameter("l_from")
l_to = pick.parameter("l_to")

pick.add_precondition(robot_at(l_from))
pick.add_precondition(reachable(l_from, l_to))
pick.add_precondition(thing_at(p, l_to))
pick.add_precondition(in_hand(tool))
p_var = Variable("p_var", Part)
pick.add_precondition(Forall(Not(in_tool(p_var, tool)), p_var))
pick.add_precondition(interactive(p, tool))

pick.add_effect(robot_at(l_from), False)
pick.add_effect(robot_at(l_to), True)
pick.add_effect(thing_at(p, l_to), False)
pick.add_effect(in_tool(p, tool), True)

# # * place
# place = unified_planning.model.InstantaneousAction(
#     "place", p=Part, tool=Tool, l_from=Location, l_to=Location
# )
# p = place.parameter("p")
# tool = place.parameter("tool")
# l_from = place.parameter("l_from")
# l_to = place.parameter("l_to")

# place.add_precondition(robot_at(l_from))
# place.add_precondition(reachable(l_from, l_to))
# place.add_precondition(in_tool(p, tool))
# place.add_precondition(in_hand(tool))

# place.add_effect(robot_at(l_from), False)
# place.add_effect(robot_at(l_to), True)
# place.add_effect(in_tool(p, tool), False)
# place.add_effect(thing_at(p, l_to), True)


# # * push, can be move


# # * insert
# insert = unified_planning.model.InstantaneousAction(
#     "insert", p=Part, tool=Tool, l_from=Location, l_to=Location
# )
# p = insert.parameter("p")
# tool = insert.parameter("tool")
# l_from = insert.parameter("l_from")
# l_to = insert.parameter("l_to")

# insert.add_precondition(reachable(l_from, l_to))
# insert.add_precondition(robot_at(l_from))
# insert.add_precondition(in_hand(tool))
# insert.add_precondition(in_tool(p, tool))
# # insert.add_precondition(interactive(p, tool))

# insert.add_effect(robot_at(l_from), False)
# insert.add_effect(robot_at(l_to), True)
# insert.add_effect(in_tool(p, tool), False)
# insert.add_effect(thing_at(p, l_to), True)
# insert.add_effect(inserted(p), True)

# # * pull
# pull = unified_planning.model.InstantaneousAction(
#     "pull", p=Part, tool=Tool, l_from=Location, l_to=Location
# )
# p = pull.parameter("p")
# tool = pull.parameter("tool")
# l_from = pull.parameter("l_from")
# l_to = pull.parameter("l_to")

# pull.add_precondition(reachable(l_from, l_to))
# pull.add_precondition(robot_at(l_from))
# pull.add_precondition(thing_at(p, l_to))
# pull.add_precondition(inserted(p))
# pull.add_precondition(in_hand(tool))
# pull.add_precondition(interactive(p, tool))

# pull.add_effect(robot_at(l_from), False)
# pull.add_effect(robot_at(l_to), True)
# pull.add_effect(inserted(p), False)
# pull.add_effect(thing_at(p, l_to), False)
# pull.add_effect(in_tool(p, tool), True)

# # * screw
# screw = unified_planning.model.InstantaneousAction(
#     "screw", p=Part, tool=Tool, l_from=Location, l_to=Location
# )
# p = screw.parameter("p")
# tool = screw.parameter("tool")
# l_from = screw.parameter("l_from")
# l_to = screw.parameter("l_to")

# screw.add_precondition(reachable(l_from, l_to))
# screw.add_precondition(robot_at(l_from))
# screw.add_precondition(in_hand(tool))
# screw.add_precondition(in_tool(p, tool))

# screw.add_effect(robot_at(l_from), False)
# screw.add_effect(robot_at(l_to), True)
# screw.add_effect(in_tool(p, tool), False)
# screw.add_effect(thing_at(p, l_to), True)
# screw.add_effect(screwed(p), True)

# # * unscrew
# unscrew = unified_planning.model.InstantaneousAction(
#     "unscrew", p=Part, tool=Tool, l_from=Location, l_to=Location
# )
# p = unscrew.parameter("p")
# tool = unscrew.parameter("tool")
# l_from = unscrew.parameter("l_from")
# l_to = unscrew.parameter("l_to")

# unscrew.add_precondition(reachable(l_from, l_to))
# unscrew.add_precondition(robot_at(l_from))
# unscrew.add_precondition(thing_at(p, l_to))
# unscrew.add_precondition(screwed(p))
# unscrew.add_precondition(in_hand(tool))
# unscrew.add_precondition(interactive(p, tool))

# unscrew.add_effect(robot_at(l_from), False)
# unscrew.add_effect(robot_at(l_to), True)
# unscrew.add_effect(screwed(p), False)
# unscrew.add_effect(thing_at(p, l_to), False)
# unscrew.add_effect(in_tool(p, tool), True)

######################################## Problem ########################################

problem = Problem("kios_tamp_problem")

########################## * knowledge

problem.add_fluent(robot_at, default_initial_value=False)
problem.add_fluent(reachable, default_initial_value=True)
problem.add_fluent(thing_at, default_initial_value=False)
problem.add_fluent(in_hand, default_initial_value=False)
problem.add_fluent(in_tool, default_initial_value=False)
problem.add_fluent(inserted, default_initial_value=False)
problem.add_fluent(screwed, default_initial_value=False)
problem.add_fluent(placed, default_initial_value=False)
problem.add_fluent(interactive, default_initial_value=False)

problem.add_action(move)
# problem.add_action(pick)
# problem.add_action(place)
# problem.add_action(insert)
# problem.add_action(pull)
# problem.add_action(screw)
# problem.add_action(unscrew)
problem.add_action(load_tool)
problem.add_action(unload_tool)

########################  * instances
# locations
# NLOC = 10
# locations = [unified_planning.model.Object("l%s" % i, Location) for i in range(NLOC)]
# problem.add_objects(locations)
# TODO: add locations
# ! for test
location_1 = unified_planning.model.Object("location_1", Location)
location_2 = unified_planning.model.Object("location_2", Location)
location_3 = unified_planning.model.Object("location_3", Location)
location_4 = unified_planning.model.Object("location_4", Location)

# tools
parallel_box1 = unified_planning.model.Object("parallel_box1", Tool)
parallel_box2 = unified_planning.model.Object("parallel_box2", Tool)
inward_claw = unified_planning.model.Object("inward_claw", Tool)
outward_claw = unified_planning.model.Object("outward_claw", Tool)
no_tool = unified_planning.model.Object("no_tool", Tool)

# parts
# * chair
leg1 = unified_planning.model.Object("leg1", Part)
leg2 = unified_planning.model.Object("leg2", Part)
nut1 = unified_planning.model.Object("nut1", Part)
nut2 = unified_planning.model.Object("nut2", Part)
seat = unified_planning.model.Object("seat", Part)
back = unified_planning.model.Object("back", Part)

# * lamp
base = unified_planning.model.Object("base", Part)
blub = unified_planning.model.Object("blub", Part)
lamp = unified_planning.model.Object("lamp", Part)

# * gearset
gear1 = unified_planning.model.Object("gear1", Part)
gear2 = unified_planning.model.Object("gear2", Part)
gear3 = unified_planning.model.Object("gear3", Part)
shaft1 = unified_planning.model.Object("shaft1", Part)
shaft2 = unified_planning.model.Object("shaft2", Part)

# * tool --- part
# * chair

problem.add_objects([location_1, location_2, location_3, location_4])
problem.add_objects([no_tool, parallel_box1, parallel_box2, inward_claw, outward_claw])

########################## * initial state

##########! for test

problem.set_initial_value(robot_at(location_1), True)
problem.set_initial_value(in_hand(parallel_box1), True)


##########!

# problem.set_initial_value(robot_at(locations[0]), True)
# for i in range(NLOC - 1):
#     problem.set_initial_value(connected(locations[i], locations[i + 1]), True)


########################## * goal
problem.add_goal(robot_at(location_4))
problem.add_goal(in_hand(no_tool))
print(problem)

# with OneshotPlanner(name="pyperplan") as planner:
#     result = planner.solve(problem)
#     if result.status == up.engines.PlanGenerationResultStatus.SOLVED_SATISFICING:
#         print("Pyperplan returned: %s" % result.plan)
#     else:
#         print("No plan found.")

with OneshotPlanner(problem_kind=problem.kind) as planner:
    result = planner.solve(problem)
    print("%s returned: %s" % (planner.name, result.plan))

# from unified_planning.engines import CompilationKind

# with Compiler(
#     problem_kind=problem.kind,
#     compilation_kind=CompilationKind.QUANTIFIERS_REMOVING,
# ) as quantifiers_remover:
#     # After we have the compiler, we get the compilation result
#     qr_result = quantifiers_remover.compile(
#         problem, CompilationKind.QUANTIFIERS_REMOVING
#     )
#     qr_problem = qr_result.problem
#     qr_kind = qr_problem.kind
