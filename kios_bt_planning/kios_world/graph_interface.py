"""
very simple interface to model the objects in the world and the states of them.
"""

import logging

import networkx as nx
from typing import Set, Dict, List, Any

from kios_world.neo4j_interface import Neo4jInterface
from kios_world.data_types import WorldNode, Relationship

from kios_utils.bblab_utils import setup_logger

"""
this is the current state holder of the world
"""

gi_logger = setup_logger(__name__, logging.DEBUG)


class GraphInterface:
    def __init__(self):
        self.nodes: Dict[str, WorldNode] = {}
        self.relations: Set[Relationship] = set()
        self.graph = nx.DiGraph()

        self.neo4j = Neo4jInterface()
        self.neo4j.close_driver()

    def add_node(self, node_name: str, raise_error=False):
        """
        add a node to the graph.
        if the node already exists, do nothing.
        """
        if node_name in self.nodes:
            return  # do nothing if node already exists
        self.nodes[node_name] = WorldNode(node_name)

    def add_properties(self, node_name: str, props: List[str]):
        """
        add a list of properties to a node.
        if the node does not exist, create it.
        if the property already exists, do nothing.
        """
        self.add_node(node_name)  # create node if it does not exist
        # * list -> set
        prop_set = set(props)
        self.nodes[node_name].update_properties(prop_set)

    def add_property(self, node_name: str, prop: str):
        self.add_node(node_name)
        self.nodes[node_name].add_property(prop)

    # * update the property status
    def update_property(self, node_name: str, prop: str, status: bool):
        self.add_node(node_name)
        if status:
            self.nodes[node_name].add_property(prop)
        else:
            self.nodes[node_name].discard_property(prop)

    def remove_property(self, node_name: str, prop: str):
        if node_name not in self.nodes:
            self.add_node(node_name)
            return  # do nothing if node does not exist
        self.nodes[node_name].discard_property(prop)

    def remove_properties(self, node_name: str, props: List[str]):
        if node_name not in self.nodes:
            self.add_node(node_name)  # create node if it does not exist
            return  # do nothing if node does not exist
        # * list -> set
        prop_set = set(props)
        self.nodes[node_name].difference_update_properties(prop_set)

    def get_node(self, node_name: str) -> WorldNode:
        if node_name not in self.nodes:
            return None  # do nothing if node does not exist
        return self.nodes[node_name]

    def update_relation(self, source: str, name: str, target: str, status: bool):
        if status:
            self.add_relation(source, name, target)
        else:
            self.remove_relation(source, name, target)

    def add_relation(self, source: str, name: str, target: str, isConstraint=False):
        """
        add a relation between two objects.
        if the objects do not exist, create them.
        if the relation already exists, do nothing.
        """
        source_node = self.get_node(source)
        target_node = self.get_node(target)
        # ! think twice. should the inconsistency be handled here?
        if source_node is None:
            # raise ValueError(f"Object {source} does not exist in the database!")
            self.add_node(source)
        if target_node is None:
            # raise ValueError(f"Object {target} does not exist in the database!")
            self.add_node(target)
        rel = Relationship(source, name, target, isConstraint=isConstraint)
        self.relations.add(rel)

    def remove_relation(self, source: str, name: str, target: str):
        rel = Relationship(source, name, target)
        self.relations.discard(rel)

    # for condition node checking
    def check_property(self, node_name: str, prop: str, status: bool = True) -> bool:
        if node_name not in self.nodes:
            return not status
        return self.nodes[node_name].check_property(prop) == status

    def check_relation(
        self, source: str, name: str, target: str, status: bool = True
    ) -> bool:
        rel = Relationship(source, name, target)
        return (rel in self.relations) == status

    def discard_relation(self, source: str, name: str, target: str):
        rel = Relationship(source, name, target)
        self.relations.discard(rel)

    def clear(self):
        self.nodes = {}
        self.relations = set()
        # self.graph = nx.DiGraph()

    def from_json(self, json_data: Dict[str, Any]):
        # * add nodes
        for node in json_data["objects"]:
            self.add_node(node["name"])
            self.add_properties(node["name"], node["properties"])

        if json_data.get("constraints") is None:
            gi_logger.warning("No constraint found in the JSON data.")
        else:
            # * add constraints (unchangeable relations)
            for constraint in json_data["constraints"]:
                self.add_relation(
                    constraint["source"],
                    constraint["name"],
                    constraint["target"],
                    isConstraint=True,
                )

        # * add relations
        if json_data.get("relations") is None:
            gi_logger.warning("No relation found in the JSON data.")
        else:
            for relation in json_data["relations"]:
                self.add_relation(
                    relation["source"], relation["name"], relation["target"]
                )

    def to_json(self) -> Dict[str, Any]:
        json_data = {"objects": [], "constraints": [], "relations": []}

        # * add nodes
        for node_name, node in self.nodes.items():
            node_data = {
                "name": node_name,
                "properties": list(node.properties),
            }  # ! BBFIX 20022024
            json_data["objects"].append(node_data)

        # * add constraints and relations (unchangeable relations)
        for rel in self.relations:
            if rel.isConstraint:
                constraint_data = {
                    "source": rel.objects[0],
                    "name": rel.relation_name,
                    "target": rel.objects[1],
                }
                json_data["constraints"].append(constraint_data)
            else:
                relation_data = {
                    "source": rel.objects[0],
                    "name": rel.relation_name,
                    "target": rel.objects[1],
                }
                json_data["relations"].append(relation_data)

        return json_data

    def refresh_neo4j(self):
        self.neo4j.open_driver()
        self.neo4j.clear_database()
        self.neo4j.create_objects(list(self.nodes.keys()))
        self.neo4j.set_properties(self.nodes)
        self.neo4j.create_relations(self.relations)
        self.neo4j.close_driver()

    def refresh_networkx(self):
        self.graph.clear()
        for node_name, node in self.nodes.items():
            self.graph.add_node(node_name, properties=node.properties)
        for rel in self.relations:
            self.graph.add_edge(
                rel.objects[0], rel.objects[1], relation=rel.relation_name
            )
