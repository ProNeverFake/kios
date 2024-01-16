"""
very simple interface to model the objects in the world and the states of them.
"""
import networkx as nx
from typing import Set, Dict


class WorldNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self.states = set()

    def update_states(self, states: Set[str]):
        self.states.update(states)

    def difference_update_states(self, states: Set[str]):
        self.states.difference_update(states)

    def __repr__(self):
        return f"{self.name}: {', '.join(self.states)}"


class Relationship:
    """
    directed relationship from source to target
    """

    def __init__(self, source, name, target):
        self.name = name
        self.objects = [source, target]

    def __repr__(self):
        return f"{self.objects[0]} {self.name} {self.objects[1]}"


class GraphInterface:
    def __init__(self):
        self.objects: Dict[str, WorldNode] = {}
        self.graph = nx.DiGraph()

    def __repr__(self):
        print_str = ""
        for obj in self.objects.values():
            print_str += f"{obj}\n"

        return print_str

    def add_node(self, object_name: str, states: Set[str] = None):
        if object_name not in self.objects:
            self.objects[object_name] = WorldNode(object_name)
            if states is not None:
                self.objects[object_name].update_states(states)
        else:
            raise ValueError(f"Object {object_name} already exists!")

    def add_states(self, object_name: str, states: Set[str]):
        if object_name not in self.objects:
            raise ValueError(f"Object {object_name} does not exist!")
        self.objects[object_name].update_states(states)

    def remove_states(self, object_name: str, states: Set[str]):
        if object_name not in self.objects:
            raise ValueError(f"Object {object_name} does not exist!")
        self.objects[object_name].difference_update_states(states)

    def get_object(self, object_name: str) -> WorldNode:
        if object_name not in self.objects:
            raise ValueError(f"Object {object_name} does not exist!")
        return self.objects[object_name]

    def update_neo4j(self):
        raise NotImplementedError


def test_world_object():
    print("test_world_object ----------------------------------")
    obj = WorldNode("obj")
    obj.update_states({"state1", "state2"})
    print(obj)
    obj.difference_update_states(
        {"state1"}
    )  # Remove individual state instead of a set of states
    print(obj)


def test_relationship():
    print("test_relationship ----------------------------------")
    rel = Relationship("ball", "on", "desk")
    print(rel)


def test_object_depot():
    print("test_object_depot ----------------------------------")
    depot = GraphInterface()
    depot.add_node("ball", {"state1", "state2"})
    depot.add_node("desk", {"state1", "state2"})
    depot.add_node("cup", {"state1", "state2"})
    depot.add_node("table", {"state1", "state2"})
    print(depot)
    depot.add_states("ball", {"state3"})
    print(depot)
    depot.remove_states("ball", {"state3"})
    print(depot)
    print(depot.get_object("ball"))


if __name__ == "__main__":
    test_world_object()
    test_relationship()
    test_object_depot()
