from kios_domain.pddl_interface import PddlInterface

# from kios_domain.problem1 import problem as problem1

# from kios_domain.problem2 import problem as problem2

# from kios_domain.problem3 import problem as problem3
from kios_domain.problem4 import problem as problem4

import os

"""
use this to generate the domain and problem files.
"""


def test_from_up():
    print("test_from_up ----------------------------------")
    pddl_interface = PddlInterface()

    pddl_interface.setup_from_up(problem4)

    pddl_interface.write_pddl()

    pddl_interface.print_domain()

    pddl_interface.print_problem()


def test_from_pddl():  # ! this suffers from the domain-problem inconsistency problem now
    print("test_from_pddl ----------------------------------")

    pddl_interface = PddlInterface()

    pddl_interface.setup_from_pddl(
        domain_file_name="domain.pddl", problem_file_name="problem.pddl"
    )

    pddl_interface.print_domain()

    pddl_interface.print_problem()

    print(pddl_interface.get_pddl_string())


if __name__ == "__main__":
    test_from_up()
    # test_from_pddl()
