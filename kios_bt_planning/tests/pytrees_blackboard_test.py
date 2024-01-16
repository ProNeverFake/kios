"""
test the functionality of the blackboard in pytrees
"""
import py_trees

blackboard = py_trees.blackboard.Client(name="Client")
blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="bar", access=py_trees.common.Access.READ)
blackboard.foo = [["entity1", "entity2"], ["entity3", "entity4"]]
print(blackboard)
