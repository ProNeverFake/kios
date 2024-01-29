from unified_planning.shortcuts import *

######################################## Type ########################################

Thing = UserType("Thing")
Tool = UserType("Tool", father=Thing)
Part = UserType("Part", father=Thing)
Hand = UserType("Hand", father=Thing)

######################################## Fluent ########################################

# * property
is_free = Fluent("is_free", BoolType(), p=Thing)
is_equippable = Fluent("is_equippable", BoolType(), tool=Tool)

# * relation
can_manipulate = Fluent("can_manipulate", BoolType(), tool=Tool, part=Part)
can_insert_to = Fluent("can_insert_to", BoolType(), part1=Part, part2=Part)
can_screw_to = Fluent("can_screw_to", BoolType(), part1=Part, part2=Part)
can_place_to = Fluent("can_place_to", BoolType(), part1=Part, part2=Part)
hold = Fluent("hold", BoolType(), thing1=Thing, thing2=Thing)
is_inserted_to = Fluent("is_inserted_to", BoolType(), part1=Part, part2=Part)
is_screwed_to = Fluent("is_screwed_to", BoolType(), part1=Part, part2=Part)
is_placed_to = Fluent("is_placed_to", BoolType(), part1=Part, part2=Part)

######################################## Action ########################################
# * load_tool
load_tool = unified_planning.model.InstantaneousAction(
    "load_tool",
    hand=Hand,
    tool=Tool,
)

tool = load_tool.parameter("tool")
hand = load_tool.parameter("hand")

load_tool.add_precondition(is_equippable(tool))
load_tool.add_precondition(is_free(hand))

load_tool.add_effect(is_free(hand), False)
load_tool.add_effect(is_equippable(tool), False)
load_tool.add_effect(hold(hand, tool), True)

# * unload_tool
unload_tool = unified_planning.model.InstantaneousAction(
    "unload_tool",
    hand=Hand,
    tool=Tool,
)

tool = unload_tool.parameter("tool")
hand = unload_tool.parameter("hand")

unload_tool.add_precondition(hold(hand, tool))
unload_tool.add_precondition(is_free(tool))

unload_tool.add_effect(is_free(hand), True)
unload_tool.add_effect(is_equippable(tool), True)
unload_tool.add_effect(hold(hand, tool), False)

# * pick_up
pick_up = unified_planning.model.InstantaneousAction(
    "pick_up",
    hand=Hand,
    tool=Tool,
    part=Part,
)

part = pick_up.parameter("part")
tool = pick_up.parameter("tool")
hand = pick_up.parameter("hand")

pick_up.add_precondition(is_free(tool))
pick_up.add_precondition(hold(hand, tool))
pick_up.add_precondition(can_manipulate(tool, part))

pick_up.add_effect(hold(tool, part), True)
pick_up.add_effect(is_free(tool), False)

# * put_down
put_down = unified_planning.model.InstantaneousAction(
    "put_down",
    hand=Hand,
    tool=Tool,
    part=Part,
)

part = put_down.parameter("part")
tool = put_down.parameter("tool")
hand = put_down.parameter("hand")

put_down.add_precondition(hold(tool, part))
put_down.add_precondition(hold(hand, tool))

put_down.add_effect(hold(hand, part), False)
put_down.add_effect(is_free(tool), True)


# * place
place = unified_planning.model.InstantaneousAction(
    "place",
    hand=Hand,
    tool=Tool,
    part1=Part,
    part2=Part,
)

part1 = place.parameter("part1")
part2 = place.parameter("part2")
tool = place.parameter("tool")
hand = place.parameter("hand")

place.add_precondition(hold(hand, tool))
place.add_precondition(hold(tool, part1))
place.add_precondition(can_place_to(part1, part2))

place.add_effect(hold(tool, part1), False)
place.add_effect(is_free(tool), True)
place.add_effect(is_placed_to(part1, part2), True)

# * detach
detach = unified_planning.model.InstantaneousAction(
    "detach",
    hand=Hand,
    tool=Tool,
    part1=Part,
    part2=Part,
)

part1 = detach.parameter("part1")
part2 = detach.parameter("part2")
tool = detach.parameter("tool")
hand = detach.parameter("hand")

detach.add_precondition(hold(hand, tool))
detach.add_precondition(is_free(tool))
detach.add_precondition(can_manipulate(tool, part1))
detach.add_precondition(is_placed_to(part1, part2))

detach.add_effect(hold(tool, part1), True)
detach.add_effect(is_free(tool), False)
detach.add_effect(is_placed_to(part1, part2), False)

# * push, can be move
# ! implement later

# * insert
insert = unified_planning.model.InstantaneousAction(
    "insert",
    hand=Hand,
    tool=Tool,
    part1=Part,
    part2=Part,
)

hand = insert.parameter("hand")
part1 = insert.parameter("part1")
part2 = insert.parameter("part2")
tool = insert.parameter("tool")

insert.add_precondition(hold(hand, tool))
insert.add_precondition(hold(tool, part1))
insert.add_precondition(can_insert_to(part1, part2))

insert.add_effect(hold(tool, part1), False)
insert.add_effect(is_free(tool), True)
insert.add_effect(is_inserted_to(part1, part2), True)

# * pull-out
pullout = unified_planning.model.InstantaneousAction(
    "pull",
    hand=Hand,
    tool=Tool,
    part1=Part,
    part2=Part,
)

part1 = pullout.parameter("part1")
part2 = pullout.parameter("part2")
tool = pullout.parameter("tool")
hand = pullout.parameter("hand")

pullout.add_precondition(hold(hand, tool))
pullout.add_precondition(is_free(tool))
pullout.add_precondition(is_inserted_to(part1, part2))
pullout.add_precondition(can_manipulate(tool, part1))

pullout.add_effect(hold(tool, part1), True)
pullout.add_effect(is_free(tool), False)
pullout.add_effect(is_inserted_to(part1, part2), False)

# * screw
screw = unified_planning.model.InstantaneousAction(
    "screw",
    hand=Hand,
    tool=Tool,
    part1=Part,
    part2=Part,
)

part1 = screw.parameter("part1")
part2 = screw.parameter("part2")
tool = screw.parameter("tool")
hand = screw.parameter("hand")

screw.add_precondition(hold(hand, tool))
screw.add_precondition(hold(tool, part1))
screw.add_precondition(can_screw_to(part1, part2))

screw.add_effect(hold(tool, part1), False)
screw.add_effect(is_free(tool), True)
screw.add_effect(is_screwed_to(part1, part2), True)

# * unscrew
unscrew = unified_planning.model.InstantaneousAction(
    "unscrew",
    hand=Hand,
    tool=Tool,
    part1=Part,
    part2=Part,
)

part1 = unscrew.parameter("part1")
part2 = unscrew.parameter("part2")
tool = unscrew.parameter("tool")
hand = unscrew.parameter("hand")

unscrew.add_precondition(hold(hand, tool))
unscrew.add_precondition(is_free(tool))
unscrew.add_precondition(is_screwed_to(part1, part2))
unscrew.add_precondition(can_manipulate(tool, part1))

unscrew.add_effect(hold(tool, part1), True)
unscrew.add_effect(is_free(tool), False)
unscrew.add_effect(is_screwed_to(part1, part2), False)

######################################## Problem ########################################

# problem = Problem("kios_tamp_problem")

# ########################## * knowledge

# # * property
# problem.add_fluent(is_free, default_initial_value=True)
# problem.add_fluent(is_equippable, default_initial_value=True)

# # * relation
# problem.add_fluent(can_manipulate, default_initial_value=False)
# problem.add_fluent(can_insert_to, default_initial_value=False)
# problem.add_fluent(can_screw_to, default_initial_value=False)
# problem.add_fluent(can_place_to, default_initial_value=False)
# problem.add_fluent(hold, default_initial_value=False)
# problem.add_fluent(is_inserted_to, default_initial_value=False)
# problem.add_fluent(is_screwed_to, default_initial_value=False)
# problem.add_fluent(is_placed_to, default_initial_value=False)

# # * action

# problem.add_action(pick_up)
# problem.add_action(put_down)
# problem.add_action(place)
# problem.add_action(detach)
# problem.add_action(insert)
# problem.add_action(pullout)
# problem.add_action(screw)
# problem.add_action(unscrew)
# problem.add_action(load_tool)
# problem.add_action(unload_tool)

# ########################* instances ########################################
# # hand instances
# left_hand = unified_planning.model.Object("left_hand", Hand)

# # tool instances
# parallel_box1 = unified_planning.model.Object("parallel_box1", Tool)
# parallel_box2 = unified_planning.model.Object("parallel_box2", Tool)
# inward_claw = unified_planning.model.Object("inward_claw", Tool)
# outward_claw = unified_planning.model.Object("outward_claw", Tool)
# no_tool = unified_planning.model.Object("no_tool", Tool)

# # part instances
# # * chair
# leg1 = unified_planning.model.Object("leg1", Part)
# leg2 = unified_planning.model.Object("leg2", Part)
# nut1 = unified_planning.model.Object("nut1", Part)
# nut2 = unified_planning.model.Object("nut2", Part)
# seat = unified_planning.model.Object("seat", Part)
# back = unified_planning.model.Object("back", Part)

# # * lamp
# base = unified_planning.model.Object("base", Part)
# blub = unified_planning.model.Object("blub", Part)
# lamp = unified_planning.model.Object("lamp", Part)

# # * gearset
# gear1 = unified_planning.model.Object("gear1", Part)
# gear2 = unified_planning.model.Object("gear2", Part)
# gear3 = unified_planning.model.Object("gear3", Part)
# shaft1 = unified_planning.model.Object("shaft1", Part)
# shaft2 = unified_planning.model.Object("shaft2", Part)
# base = unified_planning.model.Object("base", Part)

# ########################## * current problem context ######################
# # add all the related instances to the problem here

# # * hand instances
# problem.add_object(left_hand)

# # * tool instances
# problem.add_object(parallel_box1)
# problem.add_object(parallel_box2)
# problem.add_object(inward_claw)
# problem.add_object(outward_claw)
# problem.add_object(no_tool)

# # * part instances
# # * chair
# # not this time
# # * lamp
# # not this time
# # * gearset
# problem.add_object(gear1)
# problem.add_object(gear2)
# problem.add_object(gear3)
# problem.add_object(shaft1)
# problem.add_object(shaft2)
# problem.add_object(base)

# ########################## * problem constraints ##########################

# ############ * tools' manipulability
# problem.set_initial_value(can_manipulate(parallel_box1, gear1), True)
# problem.set_initial_value(can_manipulate(outward_claw, gear2), True)
# problem.set_initial_value(can_manipulate(inward_claw, gear3), True)
# problem.set_initial_value(can_manipulate(parallel_box2, shaft1), True)
# problem.set_initial_value(can_manipulate(no_tool, shaft2), True)

# ############ * assembly constraints
# # * chair
# # not implemented yet
# # * lamp
# # not implemented yet
# # * gearset
# problem.set_initial_value(can_insert_to(shaft1, base), True)
# problem.set_initial_value(can_insert_to(shaft2, base), True)
# problem.set_initial_value(can_insert_to(gear3, shaft2), True)
# problem.set_initial_value(can_insert_to(gear2, base), True)
# problem.set_initial_value(can_insert_to(gear1, shaft1), True)

# ########################## * initial world state ##########################
# # ! BBNOTE: LLM CONSENTRATION FOR STATE KEEPING LOAD SAVING
# # ! Default initial value: see fluent setup part above


# # problem.set_initial_value(hold(left_hand, parallel_box2), True)
# # problem.set_initial_value(hold(parallel_box2, shaft1), True)

# ########################## * goal ##########################################

# problem.add_goal(is_inserted_to(shaft1, base))
# print(problem)


# # with OneshotPlanner(name="pyperplan") as planner:
# #     result = planner.solve(problem)
# #     if result.status == up.engines.PlanGenerationResultStatus.SOLVED_SATISFICING:
# #         print("Pyperplan returned: %s" % result.plan)
# #     else:
# #         print("No plan found.")

# with OneshotPlanner(problem_kind=problem.kind) as planner:
#     result = planner.solve(problem)
#     print("%s returned: %s" % (planner.name, result.plan))
