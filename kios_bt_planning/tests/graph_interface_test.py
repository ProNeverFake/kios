from kios_world.graph_interface import GraphInterface
from kios_world.data_types import Relationship, WorldNode
import networkx as nx


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


def test_from_json():
    json_data = {
        "objects": [
            {"name": "table", "properties": ["state1", "state2"]},
            {"name": "ball", "properties": ["state1", "state2"]},
            {"name": "cabinet", "properties": ["open"]},
        ],
        "relations": [
            {"source": "ball", "name": "on_something", "target": "table"},
            {"source": "ball", "name": "on_something", "target": "cabinet"},
            {"source": "cabinet", "name": "on_something", "target": "table"},
            {"source": "cabinet", "name": "on_something", "target": "ball"},
        ],
    }
    gi = GraphInterface()
    gi.from_json(json_data)
    print(gi.nodes)
    print(gi.relations)


if __name__ == "__main__":
    # test_world_object()
    # test_relationship()
    # test_graph_interface()
    # test_networkx_and_neo4j()
    test_from_json()
