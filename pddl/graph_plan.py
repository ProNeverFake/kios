from planning_graph.planning_graph import PlanningGraph

planning_graph = PlanningGraph('test_domain.pddl', 
                               'test_problem.pddl',
                               visualize=True)

graph = planning_graph.create(max_num_of_levels=10)
graph.visualize_png("generated_graph.png")