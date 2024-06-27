from typing import Set


class WorldNode:
    name: str # the name of the object
    properties: set[str] # a set of the properties of the object. are just predicates.
    # many of the functions for the set "properties" are based on the set class in python
    # you should learn about the set class in python to understand how this class works

    def __init__(self, name: str) -> None:
        self.name = name
        self.properties = set() 

    def update_properties(self, prop_set: Set[str]):
        self.properties.update(prop_set)

    def add_property(self, prop: str):
        self.properties.add(prop)

    def difference_update_properties(self, prop_set: Set[str]):
        self.properties.difference_update(prop_set)

    def discard_property(self, prop: str):
        self.properties.discard(prop)

    def check_property(self, prop: str) -> bool:
        if prop in self.properties:
            return True
        else:
            return False

    def __repr__(self):
        # return f"{self.name}: {', '.join(self.properties)}"
        return f"{', '.join(self.properties)}"


class Relationship:
    """
    directed relationship from source to target
    """
    relation_name: str
    objects: list[WorldNode] # a list of the TWO objects that are related. obj1 -> obj2
    isConstraint: bool # ? is this used?

    def __init__(self, source, relation_name, target, isConstraint=False):
        self.relation_name = relation_name
        self.objects = [source, target]
        self.isConstraint = isConstraint

    def __repr__(self):
        return f"{self.objects[0]} {self.relation_name} {self.objects[1]}"

    def __eq__(self, other):
        if not isinstance(other, Relationship):
            # * BB suggest that this should be an assertion
            print("invalid comparison! not a relationship!")
            return False
        return (  
            self.relation_name == other.relation_name
            and self.objects[0] == other.objects[0]
            and self.objects[1] == other.objects[1]
        )

    def __hash__(self):
        '''
        this is for comparison. but not used above.
        '''
        return hash((self.relation_name, self.objects[0], self.objects[1]))
