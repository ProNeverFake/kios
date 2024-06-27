from typing import Any, Dict
import py_trees
from kios_bt.data_types import (
    Action,
    Condition,
    ObjectProperty,
)

from kios_world.neo4j_interface import Neo4jInterface
from kios_world.graph_interface import GraphInterface


class WorldInterface:
    neo4j: Neo4jInterface = None # this is for visualization in neo4j
    blackboard: py_trees.blackboard.Client = None # currently the blackboard from pytrees is not used
    graph_interface: GraphInterface = None # the graph interface for the RDF-like world representation

    # this is for storing a check point of the current world state and restoring it later
    check_point: Dict[str, dict] = {} 
    

    def __init__(self, graph_interface=True, blackboard=False, neo4j=False) -> None:
        if graph_interface:
            self.graph_interface = GraphInterface()

        if neo4j:
            self.neo4j = Neo4jInterface()

        if blackboard:
            self.blackboard = py_trees.blackboard.Client(name="WorldInterface")

    def initialize(self):
        pass

    def record_check_point(self, check_point_name: str = None):
        """
        record the current world state
        """
        if check_point_name is None:
            check_point_name = "default"
        self.check_point[check_point_name] = self.graph_interface.to_json()

    def restore_check_point(self, check_point_name: str = None):
        """
        restore the world state to the check point
        """
        if check_point_name is None:
            check_point_name = "default"
        self.graph_interface.from_json(self.check_point[check_point_name])

    def load_world_from_json(self, json_data: dict):
        """
        load the world from a json file, but you need to parse it first
        """
        self.graph_interface.from_json(json_data)

    def clear_world(self):
        """
        clear the world
        """
        self.graph_interface.clear()

    def update_world(self, world: dict[str, list[dict[str, Any]]]):
        """
        update the world state, add new, do not remove any.
        """
        object_list = world.get("objects")
        constraint_list = world.get("constraints")
        relation_list = world.get("relations")
        for item in object_list:
            # self.graph_interface.add_node(item.get("name"))
            self.graph_interface.add_properties(
                item.get("name"), item.get("properties")
            )

        for item in constraint_list:
            self.graph_interface.add_relation(
                source=item.get("source"),
                target=item.get("target"),
                name=item.get("name"),
                isConstraint=True,
            )

        for item in relation_list:
            self.graph_interface.add_relation(
                source=item.get("source"),
                target=item.get("target"),
                name=item.get("name"),
                isConstraint=False,
            )

    def get_world_to_json(self) -> dict:
        """
        return the world state in json dict
        """
        return self.graph_interface.to_json()

    def take_effect(self, action: Action):
        '''
        for action node.
        call this method to make the action take effect in the world state
        '''
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

    def get_object(self, object_name: str) -> Any:
        raise NotImplementedError

    def get_objects(self, object_name: str) -> Any:
        raise NotImplementedError

    def check_condition(self, condition: Condition) -> bool:
        '''
        for condition node.
        call this method to check if a condition is satisfied in the world state.
        this method assumes that a condition node can have a list of conditions to be checked.
        if all conditions are satisfied, return True, otherwise return False.
        '''
        condition_list = condition.conditions
        for item in condition_list:
            if not self.check_property(item):
                return False

        return True

    def check_property(self, prop: ObjectProperty) -> bool:
        """
        check if a property (or a relation) is satisfied in the world state.
        """
        if prop.property_value is None:
            # this is a property
            return self.graph_interface.check_property(
                prop.object_name, prop.property_name, prop.status
            )
        else:
            # this is a relation
            return self.graph_interface.check_relation(
                prop.object_name, prop.property_name, prop.property_value, prop.status
            )

        return False
