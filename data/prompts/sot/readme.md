> THIS FILE HAS NOT BEEN FINISHED.

This file shows an iterative way to generate an entire behavior tree. It includes the following steps:

1. action sequence: generate an action sequence.
2. unit tree: generate unit trees for each action.
3. tree assembly: assembly the tree by replacing preconditions with unit trees according to the action sequence (dependency).
4. (optional)check: check the behavior tree with postorder traversal to check the logic. 


+ new: relation_analysis: analyze the relation between the actions based on precondition definitions.