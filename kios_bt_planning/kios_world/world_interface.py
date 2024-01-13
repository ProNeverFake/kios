from typing import Any
import py_trees
from kios_bt.data_types import GroundedAction, GroundedCondition, Action, Condition

from kios_world.neo4j_interface import Neo4jInterface
from kios_world.graph_interface import GraphInterface


class WorldInterface:
    neo4j: Neo4jInterface = None
    blackboard: py_trees.blackboard.Client = None
    graph_interface: GraphInterface = None

    def __init__(self, graph_interface=True, blackboard=False, neo4j=False) -> None:
        if graph_interface:
            self.graph_interface = GraphInterface()

        if neo4j:
            self.neo4j = Neo4jInterface()

        if blackboard:
            self.blackboard = py_trees.blackboard.Client(name="WorldInterface")

    def initialize(self):
        pass

    def take_effect(self, action: Action):
        to_update = action.effects["true"]
        for item in to_update:
            pass

        to_remove = action.effects["false"]

    def register_predicates(self, predicates: dict) -> None:
        """
        add this predicate to the world. do nothing if the predicate already exists
        """
        if predicates is not None:
            for key, _ in predicates.items():
                self.blackboard.register_key(
                    key=key, access=py_trees.common.Access.WRITE
                )

    def get_object(self, object_name: str) -> Any:
        raise NotImplementedError

    def get_objects(self, object_name: str) -> Any:
        raise NotImplementedError

    def set_predicates(self, predicates: dict) -> None:
        """
        "take effects"
        """
        if predicates is not None:
            try:
                for key, value in predicates.items():
                    self.blackboard.set(name=key, value=value)
            except KeyError:
                print("KeyError: %s triggered this problem!" % (key))

    def get_predicates(self) -> dict:
        raise NotImplementedError

    def query_state(self, query: dict) -> bool:
        raise NotImplementedError

    def check_condition(self, grounded_condition: GroundedCondition) -> bool:
        return True
