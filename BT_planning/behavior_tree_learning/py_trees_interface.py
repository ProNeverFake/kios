"""
Interfaces to py_trees from behavior tree strings
"""
import time
import py_trees as pt
import behavior_tree_learning.behavior_tree as behavior_tree

class PyTree(pt.trees.BehaviourTree):
    """
    A class containing a behavior tree. Inherits from the py tree BehaviorTree class.
    """
    def __init__(self, string, behaviors, world_interface=None, root=None, verbose=False):
        # pylint: disable=too-many-arguments
        if root is not None:
            self.root = root
            string = self.get_bt_from_root()
        self.bt = behavior_tree.BT(string)
        self.depth = self.bt.depth()
        self.length = self.bt.length()
        self.world_interface = world_interface
        self.verbose = verbose
        self.behaviors = behaviors
        self.failed = False
        self.timeout = False

        if root is None:
            self.root, has_children = behaviors.get_node_from_string(string[0], world_interface, self.verbose)
            string.pop(0)
        else:
            has_children = False

        super().__init__(root=self.root)
        if has_children:
            self.create_from_string(string, self.root)

    def get_bt_from_root(self):
        """
        Returns bt string (actually a list) from py tree root
        by cleaning the ascii tree from py trees
        Not complete or beautiful by any means but works for many trees
        """
        string = pt.display.ascii_tree(self.root)
        print(string)
        string = string.replace('[o] ', '')
        string = string.replace('\t', '')
        string = string.replace('-->', '')
        string = string.replace('Fallback', 'f(')
        string = string.replace('Sequence', 's(')
        bt = string.split('\n')
        bt = bt[:-1] #Remove empty element because of final newline

        prev_leading_spaces = 999999
        for i in range(len(bt) - 1, -1, -1):
            leading_spaces = len(bt[i]) - len(bt[i].lstrip(' '))
            bt[i] = bt[i].lstrip(' ')
            if leading_spaces > prev_leading_spaces:
                for _ in range(round((leading_spaces - prev_leading_spaces) / 4)):
                    bt.insert(i + 1, ')')
            prev_leading_spaces = leading_spaces

        bt_obj = behavior_tree.BT(bt)
        bt_obj.close()
        return bt_obj.bt

    def create_from_string(self, string, node):
        """
        Recursive function to generate the tree from a string
        """
        while len(string) > 0:
            if string[0] == ')':
                string.pop(0)
                return node

            newnode, has_children = self.behaviors.get_node_from_string(string[0], self.world_interface, self.verbose)
            string.pop(0)
            if has_children:
                #Node is a control node or decorator with children - add subtree via string and then add to parent
                newnode = self.create_from_string(string, newnode)
                node.add_child(newnode)
            else:
                #Node is a leaf/action node - add to parent, then keep looking for siblings
                node.add_child(newnode)

        #This return is only reached if there are too few up nodes
        return node

    def run_bt(self, max_ticks=30, max_time=30.0):
        """
        Function executing the behavior tree
        """
        ticks = 0
        max_straight_fails = 1
        straight_fails = 0
        successes_required = 2
        successes = 0
        status_ok = True
        start = time.time()

        while (self.root.status is not pt.common.Status.FAILURE or straight_fails < max_straight_fails) and \
              (self.root.status is not pt.common.Status.SUCCESS or successes < successes_required) and \
              ticks < max_ticks and status_ok:

            status_ok = self.world_interface.get_feedback() #Wait for connection

            if status_ok:
                self.root.tick_once()
                self.world_interface.send_references()

                ticks += 1
                if self.root.status is pt.common.Status.SUCCESS:
                    successes += 1
                else:
                    successes = 0

                if self.root.status is pt.common.Status.FAILURE:
                    straight_fails += 1
                else:
                    straight_fails = 0

                if time.time() - start > max_time:
                    status_ok = False
                    print("Max time expired")

        print(ticks, time.time()-start)
        if ticks >= max_ticks:
            self.timeout = True
        if straight_fails >= max_straight_fails:
            self.failed = True
        return ticks, status_ok

    def save_fig(self, path, name='Behavior tree', svg=False):
        """
        Saves the tree as a figure
        """
        if svg:
            pt.display.render_dot_tree(self.root, name=name, target_directory=path, png=False, svg=True)
        else:
            pt.display.render_dot_tree(self.root, name=name, target_directory=path)
