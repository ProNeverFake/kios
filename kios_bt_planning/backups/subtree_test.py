# from kios_bt.bt_factory import SubtreeFactory
# from kios_bt.data_types import *
# from kios_bt.pybt_io import BehaviorTreeTemplates
# from kios_utils.pybt_test import (
#     generate_bt_stewardship,
#     tick_once_test,
#     render_dot_tree,
# )

# move = ActionInstance(
#     tag="move to bedroom",
#     name="move",
#     variables={"l_from": "kitchen", "l_to": "bedroom"},
# )

# bt_templates = BehaviorTreeTemplates()
# bt_templates.initialize()

# subtree_factory = SubtreeFactory(bt_templates)

# test_tree = subtree_factory.generate_standard_subtree(move)

# bt = generate_bt_stewardship(test_tree)

# render_dot_tree(bt)

# tick_once_test(bt)
