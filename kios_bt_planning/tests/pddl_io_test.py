from kios_domain.pddl_interface import PddlInterface

from kios_domain.problem1 import problem

import os

pddl_interface = PddlInterface()

pddl_interface.setup_from_up(problem)

pddl_interface.write_pddl()

pddl_interface.print_domain()

pddl_interface.print_problem()

pddl_interface.setup_from_pddl()

pddl_interface.print_domain()

pddl_interface.print_problem()

print(pddl_interface.get_pddl_string())
