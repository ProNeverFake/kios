from kios_domain.domain_v0 import *

problem = Problem("robot_assembly_problem")

########################## * knowledge

# * property
# ! BBMOD
problem.add_fluent(is_free, default_initial_value=False)
problem.add_fluent(is_equippable, default_initial_value=False)

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
problem.add_action(load_tool)
problem.add_action(unload_tool)

########################* instances ########################################
# hand instances
left_hand = unified_planning.model.Object("left_hand", Hand)

# tool instances
parallel_box1 = unified_planning.model.Object("parallel_box1", Tool)
parallel_box2 = unified_planning.model.Object("parallel_box2", Tool)
inward_claw = unified_planning.model.Object("inward_claw", Tool)
outward_claw = unified_planning.model.Object("outward_claw", Tool)
no_tool = unified_planning.model.Object("no_tool", Tool)

# part instances
# * chair
leg1 = unified_planning.model.Object("leg1", Part)
leg2 = unified_planning.model.Object("leg2", Part)
nut1 = unified_planning.model.Object("nut1", Part)
nut2 = unified_planning.model.Object("nut2", Part)
seat = unified_planning.model.Object("seat", Part)
back = unified_planning.model.Object("back", Part)

# * lamp
lampbase = unified_planning.model.Object("lampbase", Part)
blub = unified_planning.model.Object("blub", Part)
lamp = unified_planning.model.Object("lamp", Part)

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
problem.set_initial_value(can_manipulate(parallel_box2, gear1), True)
problem.set_initial_value(can_manipulate(outward_claw, gear2), True)
problem.set_initial_value(can_manipulate(outward_claw, gear3), True)
problem.set_initial_value(can_manipulate(no_tool, shaft3), True)
problem.set_initial_value(can_manipulate(parallel_box1, shaft1), True)

############ * assembly constraints
# * chair
problem.set_initial_value(can_screw_to(leg1, seat), True)
problem.set_initial_value(can_screw_to(leg2, seat), True)
problem.set_initial_value(can_insert_to(back, seat), True)
problem.set_initial_value(can_screw_to(nut1, seat), True)
problem.set_initial_value(can_screw_to(nut2, seat), True)

# * lamp
problem.set_initial_value(can_screw_to(blub, lampbase), True)
problem.set_initial_value(can_place_to(lamp, blub), True)

# * gearset
problem.set_initial_value(can_insert_to(shaft1, gearbase_hole1), True)
problem.set_initial_value(can_insert_to(shaft3, gearbase_hole3), True)
problem.set_initial_value(can_insert_to(gear3, shaft3), True)
problem.set_initial_value(can_insert_to(gear2, shaft2), True)
problem.set_initial_value(can_insert_to(gear1, shaft1), True)
