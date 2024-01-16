# Import the PDDLReader and PDDLWriter classes
from unified_planning.io import PDDLReader, PDDLWriter

from unified_planning.shortcuts import *

import os
import os.path


class PddlInterface:
    pddl_writer: PDDLWriter = None
    pddl_reader: PDDLReader = None
    problem: Problem = None
    default_domain_path: str = None
    default_problem_path: str = None

    def __init__(
        self,
        default_domain_path: str = None,
        default_problem_path: str = None,
    ):
        if default_domain_path is None:
            self.default_domain_path = os.path.join(
                os.getcwd(),
                "src/kios/kios_bt_planning/kios_domain",
            )
        if default_problem_path is None:
            self.default_problem_path = os.path.join(
                os.getcwd(),
                "src/kios/kios_bt_planning/kios_domain",
            )

        print(
            "\033[93m################################## BB WARNING ###################################"
        )
        print("default_domain_path: ", self.default_domain_path)
        print("default_problem_path: ", self.default_problem_path)
        print("Please be aware of problem consistency:")
        print("Known error: syntax error can happen if the initial state is")
        print("using some instance that is not added into the problem. This")
        print("can always happen when the pddl file you want to parse is ")
        print("generated from unified planning problem by the pddl writer.")
        print("This won't be fixed currently. Please be aware of this.")
        print(
            "################################################################################"
        )
        print("\033[0m")  # Reset color to default

    def update_problem(self):
        pass

    def setup_from_up(self, problem: Problem):
        self.problem = problem
        self.pddl_writer = PDDLWriter(problem)
        self.pddl_reader = PDDLReader()

    def setup_from_pddl(
        self,
        domain_path: str = None,
        problem_path: str = None,
    ):
        if domain_path is None:
            domain_path = self.default_domain_path
        if problem_path is None:
            problem_path = self.default_problem_path

        self.pddl_reader = PDDLReader()

        self.problem = self.pddl_reader.parse_problem(
            os.path.join(domain_path, "domain.pddl"),
            os.path.join(problem_path, "problem.pddl"),
        )
        self.pddl_writer = PDDLWriter(self.problem)

    def write_pddl(
        self,
        domain_path: str = None,
        problem_path: str = None,
    ):
        if domain_path is None:
            domain_path = self.default_domain_path
        if problem_path is None:
            problem_path = self.default_problem_path

        if self.problem is None:
            raise Exception("Problem is not set")

        self.update_problem()
        self.pddl_writer = PDDLWriter(self.problem)
        self.pddl_writer.write_domain(os.path.join(domain_path, "domain.pddl"))
        self.pddl_writer.write_problem(os.path.join(problem_path, "problem.pddl"))

    def get_pddl_string(self) -> [str, str]:
        if self.problem is None:
            raise Exception("Problem is not set")
        self.update_problem()
        writer = PDDLWriter(self.problem)
        return writer.get_domain(), writer.get_problem()

    def print_domain(self):
        if self.problem is None:
            raise Exception("Problem is not set")
        self.update_problem()
        writer = PDDLWriter(self.problem)
        writer.print_domain()

    def print_problem(self):
        if self.problem is None:
            raise Exception("Problem is not set")
        self.update_problem()
        writer = PDDLWriter(self.problem)
        writer.print_problem()
