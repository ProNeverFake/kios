# Import the PDDLReader and PDDLWriter classes
from unified_planning.io import PDDLReader, PDDLWriter

from unified_planning.shortcuts import *

import os
import os.path
import re

from typing import List, Dict, Any, Tuple


class PddlInterface:
    pddl_writer: PDDLWriter = None
    pddl_reader: PDDLReader = None
    problem: Problem = None
    default_domain_path: str = None
    default_problem_path: str = None

    pddl_domain_file_name: str = "domain_001.pddl"
    pddl_problem_file_name: str = "problem_001.pddl"

    def __init__(
        self,
        default_domain_path: str = None,
        default_problem_path: str = None,
    ):
        if default_domain_path is None:
            self.default_domain_path = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "domains",
            )
            if not os.path.exists(self.default_domain_path):
                os.makedirs(self.default_domain_path)
        if default_problem_path is None:
            self.default_problem_path = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "problems",
            )
            if not os.path.exists(self.default_problem_path):
                os.makedirs(self.default_problem_path)

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

    def set_pddl_names(self, domain_name: str, problem_name: str):
        self.pddl_domain_file_name = domain_name
        self.pddl_problem_file_name = problem_name

    def update_problem(self):
        pass

    def setup_from_up(self, problem: Problem):
        self.problem = problem
        self.pddl_writer = PDDLWriter(problem)
        self.pddl_reader = PDDLReader()

    def setup_from_pddl(
        self,
        domain_file_name: str,
        problem_file_name: str,
        domain_path: str = None,
        problem_path: str = None,
    ):
        if domain_path is None:
            domain_path = self.default_domain_path
        if problem_path is None:
            problem_path = self.default_problem_path

        if domain_file_name is None or problem_file_name is None:
            raise Exception("YOU MUST SPECIFY THE DOMAIN AND PROBLEM FILE NAME")

        self.pddl_reader = PDDLReader()

        self.problem = self.pddl_reader.parse_problem(
            os.path.join(domain_path, self.pddl_domain_file_name),
            os.path.join(problem_path, self.pddl_problem_file_name),
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

        domain_file_path = os.path.join(domain_path, self.pddl_domain_file_name)
        problem_file_path = os.path.join(problem_path, self.pddl_problem_file_name)

        # Check if the domain and problem files already exist, if so,
        # modify the file name using regular expressions
        if os.path.exists(domain_file_path):
            # Modify the domain file name using regular expressions
            domain_file_name = self.pddl_domain_file_name
            match = re.match(r"domain_(\d+)\.pddl", domain_file_name)
            if match:
                file_number = int(match.group(1))
                while os.path.exists(
                    os.path.join(domain_path, f"domain_{file_number:03}.pddl")
                ):
                    file_number += 1
                domain_file_name = f"domain_{file_number:03}.pddl"
            else:
                domain_file_name = "domain_001.pddl"

            domain_file_path = os.path.join(domain_path, domain_file_name)

        if os.path.exists(problem_file_path):
            # Modify the problem file name using regular expressions
            problem_file_name = self.pddl_problem_file_name
            match = re.match(r"problem_(\d+)\.pddl", problem_file_name)
            if match:
                file_number = int(match.group(1))
                while os.path.exists(
                    os.path.join(problem_path, f"problem_{file_number:03}.pddl")
                ):
                    file_number += 1
                problem_file_name = f"problem_{file_number:03}.pddl"
            else:
                problem_file_name = "problem_001.pddl"

            problem_file_path = os.path.join(problem_path, problem_file_name)

        # Create the directories if they don't exist
        if not os.path.exists(domain_path):
            os.makedirs(domain_path)
        if not os.path.exists(problem_path):
            os.makedirs(problem_path)

        self.pddl_writer.write_domain(domain_file_path)
        self.pddl_writer.write_problem(problem_file_path)

    def get_pddl_string(self) -> Tuple[str, str]:
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
