from typing import Any
import py_trees
from kios_bt.data_types import (
    GroundedAction,
    GroundedCondition,
    Action,
    Condition,
    ObjectProperty,
)

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

    # for action node
    def take_effect(self, action: Action):
        # to add/update
        to_update_list = action.effects
        for item in to_update_list:
            if item.property_value is None:
                # this is a property
                self.graph_interface.update_property(
                    item.object_name, item.property_name, item.status
                )
            else:
                # this is a relation
                self.graph_interface.update_relation(
                    item.object_name,
                    item.property_name,
                    item.property_value,
                    item.status,
                )

    def register_predicates(self, predicates: dict) -> None:  # ! discard
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

    # for condition node
    def check_condition(self, condition: Condition) -> bool:
        condition_list = condition.conditions
        for item in condition_list:
            if not self.check_property(item):
                return False

        return True

    def check_property(self, prop: ObjectProperty) -> bool:
        """
        check if the property is in the same status as specified in the property object
        """
        if prop.property_value is None:
            # this is a property
            result = self.graph_interface.check_property(
                prop.object_name, prop.property_name
            )
            if result != prop.status:
                return False
        else:
            # this is a relation
            result = self.graph_interface.check_relation(
                prop.object_name, prop.property_name, prop.property_value
            )
            if result != prop.status:
                return False
        return True
