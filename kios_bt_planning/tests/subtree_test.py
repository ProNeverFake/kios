from kios_bt.subtrees import ActionInstance, SubtreeFactory

move = ActionInstance(name="move", variables={"l_from": "kitchen", "l_to": "bedroom"})

subtree_factory = SubtreeFactory()
