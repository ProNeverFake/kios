"""
very simple interface to model the objects in the world and the states of them.
"""
import networkx as nx
from typing import Set, Dict, List, Any

from kios_world.neo4j_interface import Neo4jInterface
from kios_world.data_types import WorldNode, Relationship

"""
this is the current state holder of the world
"""


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

    def add_relation(self, source: str, name: str, target: str):
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
        rel = Relationship(source, name, target)
        self.relations.add(rel)

    def remove_relation(self, source: str, name: str, target: str):
        rel = Relationship(source, name, target)
        self.relations.discard(rel)

    # for condition node checking
    def check_property(self, node_name: str, prop: str) -> bool:
        if node_name not in self.nodes:
            return False
        return self.nodes[node_name].check_property(prop)

    def check_relation(self, source: str, name: str, target: str) -> bool:
        rel = Relationship(source, name, target)
        return rel in self.relations

    def discard_relation(self, source: str, name: str, target: str):
        rel = Relationship(source, name, target)
        self.relations.discard(rel)

    def from_json(self, json_data: Dict[str, Any]):
        # * add nodes
        for node in json_data["objects"]:
            self.add_node(node["name"])
            self.add_properties(node["name"], node["properties"])

        # * add constraints (unchangeable relations)
        for constraint in json_data["constraints"]:
            self.add_relation(
                constraint["source"], constraint["name"], constraint["target"]
            )

        # * add relations
        for relation in json_data["relations"]:
            self.add_relation(relation["source"], relation["name"], relation["target"])

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
