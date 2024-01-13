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
        if node_name in self.nodes:
            return  # do nothing if node already exists
        self.nodes[node_name] = WorldNode(node_name)

    def add_properties(self, node_name: str, props: List[str]):
        self.add_node(node_name)  # create node if it does not exist
        # * list -> set
        prop_set = set(props)
        self.nodes[node_name].update_properties(prop_set)

    def add_property(self, node_name: str, prop: str):
        self.add_node(node_name)
        self.nodes[node_name].add_property(prop)

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

    def get_object(self, node_name: str) -> WorldNode:
        if node_name not in self.nodes:
            return None  # do nothing if node does not exist
        return self.nodes[node_name]

    def add_relation(self, source: str, name: str, target: str):
        source_node = self.get_object(source)
        target_node = self.get_object(target)
        if source_node is None:
            raise ValueError(f"Object {source} does not exist in the database!")
        if target_node is None:
            raise ValueError(f"Object {target} does not exist in the database!")
        rel = Relationship(source, name, target)
        self.relations.add(rel)

    def discard_relation(self, source: str, name: str, target: str):
        rel = Relationship(source, name, target)
        self.relations.discard(rel)

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


def test_world_object():
    print("test_world_object ----------------------------------")
    obj = WorldNode("ball")
    print(obj)
    obj.update_properties({"state1", "state2"})
    print(obj)
    obj.add_property("state3")
    print(obj)
    obj.difference_update_properties({"state3"})
    print(obj)
    obj.discard_property("state2")
    print(obj)


def test_relationship():
    print("test_relationship ----------------------------------")
    rel = Relationship("ball", "on", "table")
    print(rel)
    rel2 = Relationship("ball", "on", "table")
    print(rel == rel2)
    rel3 = Relationship("ball", "on", "cup")
    print(rel == rel3)
    print(rel == "ball")
    print(rel == 1)
    print(rel == [1, 2, 3])
    print(rel == {"a": 1, "b": 2})


def test_graph_interface():
    print("test_graph_interface ----------------------------------")
    graph = GraphInterface()
    graph.add_node("ball")
    graph.add_node("table")
    graph.add_node("cup")
    graph.add_properties("ball", ["state1", "state2"])
    graph.add_property("table", "state3")
    graph.add_relation("ball", "on", "table")
    graph.add_relation("ball", "on", "cup")
    graph.add_relation("cup", "on", "table")
    graph.add_relation("cup", "on", "ball")
    print(graph.nodes)
    print(graph.relations)
    graph.remove_property("ball", "state1")
    graph.remove_properties("ball", ["state2"])
    print(graph.nodes)
    graph.discard_relation("ball", "on", "cup")
    print(graph.relations)


def test_networkx_and_neo4j():
    print("test_networkx ----------------------------------")
    # Create a new directed graph
    gi = GraphInterface()
    gi.add_node("table")
    gi.add_node("ball")
    gi.add_node("cabinet")
    gi.add_properties("cabinet", ["open"])
    gi.add_relation("ball", "on_something", "table")
    gi.refresh_networkx()
    print(gi.graph.nodes)
    # visualize the graph, invluding the properties and relation names
    import matplotlib.pyplot as plt

    nx.draw(gi.graph, with_labels=True)
    # plt.show()

    print("test_neo4j ----------------------------------")
    gi.refresh_neo4j()


if __name__ == "__main__":
    test_world_object()
    test_relationship()
    test_graph_interface()
    test_networkx_and_neo4j()
