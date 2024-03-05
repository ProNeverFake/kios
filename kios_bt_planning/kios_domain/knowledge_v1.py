from kios_domain.domain_v1 import *

problem = Problem("robot_assembly_problem")

########################## * knowledge

# * property
# ! BBMOD
problem.add_fluent(is_empty, default_initial_value=False)

# * relation
problem.add_fluent(can_manipulate, default_initial_value=False)
problem.add_fluent(can_insert_to, default_initial_value=False)
problem.add_fluent(can_screw_to, default_initial_value=False)
problem.add_fluent(can_place_to, default_initial_value=False)
problem.add_fluent(hold, default_initial_value=False)
problem.add_fluent(is_inserted_to, default_initial_value=False)
problem.add_fluent(is_screwed_to, default_initial_value=False)
problem.add_fluent(is_placed_to, default_initial_value=False)

# * action

problem.add_action(pick_up)
problem.add_action(put_down)
problem.add_action(place)
problem.add_action(detach)
problem.add_action(insert)
problem.add_action(pullout)
problem.add_action(screw)
problem.add_action(unscrew)
problem.add_action(change_tool)

########################* instances ########################################
# hand instances
left_hand = unified_planning.model.Object("left_hand", Hand)

# tool instances
clampgripper = unified_planning.model.Object("clampgripper", Tool)
parallelgripper = unified_planning.model.Object("parallelgripper", Tool)
inwardgripper = unified_planning.model.Object("inwardgripper", Tool)
outwardgripper = unified_planning.model.Object("outwardgripper", Tool)
defaultgripper = unified_planning.model.Object("defaultgripper", Tool)

# part instances
# * chair
leg1 = unified_planning.model.Object("leg1", Part)
leg2 = unified_planning.model.Object("leg2", Part)
nut1 = unified_planning.model.Object("nut1", Part)
nut2 = unified_planning.model.Object("nut2", Part)
seat = unified_planning.model.Object("seat", Part)
chairback = unified_planning.model.Object("chairback", Part)

# * lamp
lampbase = unified_planning.model.Object("lampbase", Part)
lampblub = unified_planning.model.Object("lampblub", Part)
lampshade = unified_planning.model.Object("lampshade", Part)

# * cabinet

# * square table

# * round table

# * gearset
gear1 = unified_planning.model.Object("gear1", Part)
gear2 = unified_planning.model.Object("gear2", Part)
gear3 = unified_planning.model.Object("gear3", Part)

shaft1 = unified_planning.model.Object("shaft1", Part)
shaft2 = unified_planning.model.Object("shaft2", Part)
shaft3 = unified_planning.model.Object("shaft3", Part)

gearbase = unified_planning.model.Object("gearbase", Part)

gearbase_hole1 = unified_planning.model.Object("gearbase_hole1", Part)
gearbase_hole3 = unified_planning.model.Object("gearbase_hole3", Part)


########################## * problem constraints ##########################

############ * tools' manipulability
problem.set_initial_value(can_manipulate(parallelgripper, gear1), True)
problem.set_initial_value(can_manipulate(outwardgripper, gear2), True)
problem.set_initial_value(can_manipulate(outwardgripper, gear3), True)
problem.set_initial_value(can_manipulate(defaultgripper, shaft3), True)
problem.set_initial_value(can_manipulate(clampgripper, shaft1), True)

############ * assembly constraints
# * chair
problem.set_initial_value(can_screw_to(leg1, seat), True)
problem.set_initial_value(can_screw_to(leg2, seat), True)
problem.set_initial_value(can_insert_to(chairback, seat), True)
problem.set_initial_value(can_screw_to(nut1, seat), True)
problem.set_initial_value(can_screw_to(nut2, seat), True)

# * lamp
problem.set_initial_value(can_screw_to(lampblub, lampbase), True)
problem.set_initial_value(can_place_to(lampshade, lampblub), True)

# * gearset
problem.set_initial_value(can_insert_to(shaft1, gearbase_hole1), True)
problem.set_initial_value(can_insert_to(shaft3, gearbase_hole3), True)
problem.set_initial_value(can_insert_to(gear3, shaft3), True)
problem.set_initial_value(can_insert_to(gear2, shaft2), True)
problem.set_initial_value(can_insert_to(gear1, shaft1), True)
