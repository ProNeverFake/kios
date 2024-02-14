"""
see kios_world module for the world interface
"""
# ! DISCARDED
# ! BBREMOVE

from unified_planning.shortcuts import *

Location = UserType("Location")

at = Fluent("at", Location)
distance = Fluent("distance", IntType(), l1=Location, l2=Location)
battery_charge = Fluent("battery_charge", IntType(0, 100))

move = InstantaneousAction("move", l_from=Location, l_to=Location)
l_from = move.parameter("l_from")
l_to = move.parameter("l_to")
move.add_precondition(Equals(at, l_from))
move.add_effect(at, l_to)
move.add_decrease_effect(battery_charge, distance(l_from, l_to))

l1 = Object("l1", Location)
l2 = Object("l2", Location)
l3 = Object("l3", Location)
l4 = Object("l4", Location)
l5 = Object("l5", Location)
locations = [l5, l4, l3, l2, l1]

problem = Problem("moving_robot")
problem.add_fluent(at)
problem.add_fluent(battery_charge)
problem.add_fluent(distance, default_initial_value=101)
problem.add_action(move)
problem.add_objects(locations)

problem.set_initial_value(at, l1)
problem.set_initial_value(battery_charge, 100)

problem.set_initial_value(distance(l1, l2), 20)
problem.set_initial_value(distance(l2, l3), 30)
problem.set_initial_value(distance(l3, l4), 20)
problem.set_initial_value(distance(l4, l5), 30)

problem.set_initial_value(distance(l1, l3), 60)

problem.add_goal(Equals(at, l5))
battery_exp = FluentExp(battery_charge)
robot_at = FluentExp(at)

with SequentialSimulator(problem=problem) as simulator:
    initial_state = simulator.get_initial_state()
    for travel_location in locations:
        if simulator.is_applicable(initial_state, move, (l1, travel_location)):
            print(f"From l1 we can reach: {travel_location}")

    # print(initial_state)

    state_at_l3 = simulator.apply(initial_state, move, (l1, l3))
    for travel_location in locations:
        if simulator.is_applicable(state_at_l3, move, (l3, travel_location)):
            print(f"From l3 we can reach: {travel_location}")
    state_at_l4 = simulator.apply(state_at_l3, move, (l3, l4))
    if simulator.is_applicable(state_at_l4, move, (l4, l5)):
        print("Done!")
    else:
        print(f"Problem! Battery too low: {state_at_l4.get_value(battery_exp)}")

    print(state_at_l3)

    state_at_l2 = simulator.apply(initial_state, move, (l1, l2))
    new_state_at_l3 = simulator.apply(state_at_l2, move, (l2, l3))
    new_state_better = (
        new_state_at_l3.get_value(battery_exp) > state_at_l3.get_value(battery_exp)
    ).simplify()
    if new_state_better.bool_constant_value():
        print("Reaching l3 passing through l2 saves battery!")
    else:
        print("Can't save battery reaching l3, the problem has no solution!")

    state_at_l3 = new_state_at_l3
    state_at_l4 = simulator.apply(state_at_l3, move, (l3, l4))
    if simulator.is_applicable(state_at_l4, move, (l4, l5)):
        print("Done!")
    else:
        print(f"Problem! Battery too low: {state_at_l4.get_value(battery_exp)}")
    state_at_l5 = simulator.apply(state_at_l4, move, (l4, l5))

    print(state_at_l5)

    from unified_planning.plans import SequentialPlan, ActionInstance

    plan = SequentialPlan(
        [
            ActionInstance(move, (l1, l2)),
            ActionInstance(move, (l2, l3)),
            ActionInstance(move, (l3, l4)),
            ActionInstance(move, (l4, l5)),
        ]
    )
    print(
        f"Initial battery value: {initial_state.get_value(battery_exp).constant_value()}"
    )
    current_state = initial_state
    # We also store the states to plot the metrics later
    states = [current_state]
    for action_instance in plan.actions:
        current_state = simulator.apply(current_state, action_instance)
        if current_state is None:
            print(f"Error in applying: {action_instance}")
            break
        states.append(current_state)
        current_battery_value = current_state.get_value(battery_exp).constant_value()
        # in current_battery_value we inspect the State
        print(f"Battery value after {action_instance}: {current_battery_value}")
        print(f"Robot is at: {current_state.get_value(robot_at)}")

    # from unified_planning.plot import plot_sequential_plan

    # # Redefine the plot package methods imported above to print the plot to a temp file
    # # if the exception "could not locate runnable browser" is raised. This usually happens
    # # in the Continuous Integration.

    # from inspect import getmembers, isfunction
    # from unified_planning import plot
    # from functools import partial
    # import os, uuid, tempfile as tf

    # # Define the function that will be executed instead
    # def _function(original_function, *args, **kwargs):
    #     try:
    #         original_function(*args, **kwargs)
    #     except Exception as e:
    #         if "could not locate runnable browser" in str(e):
    #             original_function(
    #                 *args,
    #                 **kwargs,
    #                 filename=f"{os.path.join(tf.gettempdir(), str(uuid.uuid1()))}.png",
    #             )
    #         else:
    #             raise e

    # # Iterate over all the functions of the plot package
    # for function_name, function in getmembers(plot, isfunction):
    #     # Override the original function with the new one
    #     globals()[function_name] = partial(_function, function)

    # plot_sequential_plan(plan, problem, battery_exp, figsize=(9, 3))
