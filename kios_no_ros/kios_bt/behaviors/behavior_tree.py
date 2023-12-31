"""Class for handling string representations of behavior trees."""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import copy
import random
from typing import List, Tuple

from behaviors.behavior_lists import BehaviorLists, ParameterizedNode


class BT:
    """Class for handling list representations of behavior trees."""

    def __init__(self, bt: List[str], behavior_lists: BehaviorLists):
        """Create a BT."""
        self.bt = bt[:]
        self.behaviors = behavior_lists

    def set(self, bt: List[ParameterizedNode]):
        """Set bt list."""
        self.bt = copy.deepcopy(bt)
        return self

    def random(self, length: int, p_leaf: float) -> List[ParameterizedNode]:
        """
        Create a random bt of the given length.

        Tries to follow some of the rules for valid trees to speed up the process.

        Args:
        ----
            length: length of the BT.
            p_leaf: probability to generate a leaf node.

        """
        self.bt = []
        while not self.is_valid():
            if length == 1:
                self.bt = [self.behaviors.get_random_behavior_node()]
            else:
                self.bt = [self.behaviors.get_random_control_node()]
                for _ in range(length - 1):
                    self.bt += [self.random_node(p_leaf)]

                    if self.behaviors.is_behavior_node(self.bt[-1]):
                        self.bt += [self.behaviors.get_up_node()]

                self.close()

        return self.bt

    def is_valid(self) -> bool:
        """Check if bt is a valid behavior tree."""
        valid = True

        # Empty list
        if len(self.bt) <= 0:
            valid = False

        # The first element cannot be a leaf if after it there are other elements
        elif not self.behaviors.is_control_node(self.bt[0]) and len(self.bt) != 1:
            valid = False
        else:
            has_behavior = False
            for i in range(len(self.bt) - 1):
                # 'up' directly after a control node
                if self.behaviors.is_control_node(
                    self.bt[i]
                ) and self.behaviors.is_up_node(self.bt[i + 1]):
                    valid = False
                # Identical leaf nodes directly after one another - waste
                elif self.behaviors.is_leaf_node(self.bt[i]):
                    if self.bt[i] == self.bt[i + 1]:
                        valid = False
                    else:
                        if self.behaviors.is_behavior_node(self.bt[i]):
                            has_behavior = True
                            # condition node after behavior node. Can in some rare cases be more optimal but
                            # is never necessary and makes tree much harder to read.
                            if self.behaviors.is_condition_node(self.bt[i + 1]):
                                valid = False
                # check for non-BT elements
                elif not self.behaviors.is_valid_node(self.bt[i]):
                    valid = False

            if not has_behavior and not self.behaviors.is_behavior_node(self.bt[-1]):
                valid = False

            if valid:
                # Check on the bt depth: to be > 0
                depth = self.depth()
                if (depth < 0) or (depth == 0 and len(self.bt) > 1):
                    valid = False

            if valid and self.behaviors.is_control_node(self.bt[0]):
                fallback_allowed = True
                sequence_allowed = True
                if self.behaviors.is_fallback_node(self.bt[0]):
                    fallback_allowed = False
                elif self.behaviors.is_sequence_node(self.bt[0]):
                    sequence_allowed = False
                valid = self.is_subtree_valid(
                    self.bt[1:], fallback_allowed, sequence_allowed
                )

        return valid

    def is_subtree_valid(
        self,
        subtree: List[ParameterizedNode],
        fallback_allowed: bool,
        sequence_allowed: bool,
    ) -> bool:
        # pylint: disable=too-many-return-statements, too-many-branches
        """
        BB: Recursively check the validation of a subtree.
        Check whether the subtree starting with subtree[0] is valid according to a couple rules.

        1. Fallbacks must not be children of fallbacks.
        2. Sequences must not be children of sequences.
        """
        while len(subtree) > 0:
            node = subtree.pop(0)

            if self.behaviors.is_up_node(node):
                return True
            if node in self.behaviors.atomic_fallback_nodes:
                if not fallback_allowed:
                    return False
            elif node in self.behaviors.atomic_sequence_nodes:
                if not sequence_allowed:
                    return False
            elif self.behaviors.is_control_node(node):
                if self.behaviors.is_fallback_node(node):
                    if fallback_allowed:
                        if not self.is_subtree_valid(subtree, False, True):
                            return False
                    else:
                        return False
                elif self.behaviors.is_sequence_node(node):
                    if sequence_allowed:
                        if not self.is_subtree_valid(subtree, True, False):
                            return False
                    else:
                        return False
                else:
                    if not self.is_subtree_valid(subtree, True, True):
                        return False

        return False

    def close(self) -> None:
        """Add missing up nodes at the end, or removes from the end if too many.

        guarantee that the tree is valid
        """
        open_subtrees = 0

        # Make sure tree always ends with up node if starts with control node
        if len(self.bt) > 0:
            if self.behaviors.is_control_node(
                self.bt[0]
            ) and not self.behaviors.is_up_node(self.bt[len(self.bt) - 1]):
                self.bt += self.behaviors.get_up_node()

        for node in self.bt:
            if self.behaviors.is_control_node(node):
                open_subtrees += 1
            elif self.behaviors.is_up_node(node):
                open_subtrees -= 1

        if open_subtrees > 0:
            for _ in range(open_subtrees):
                self.bt += self.behaviors.get_up_node()
        elif open_subtrees < 0:
            for _ in range(-open_subtrees):
                # Do not remove the very last node, and only up nodes
                for j in range(len(self.bt) - 2, 0, -1):
                    # pragma: no branch, we will always find an up
                    if self.behaviors.is_up_node(self.bt[j]):
                        self.bt.pop(j)
                        break

    def trim(self) -> None:
        """Remove control nodes with only one child.

        BB: Remove useless parts of the tree.
        """
        for index in range(len(self.bt) - 1, -1, -1):
            if self.behaviors.is_control_node(self.bt[index]):
                children = self.find_children(index)
                if len(children) <= 1:
                    up_node_index = self.find_up_node(index)
                    self.bt.pop(up_node_index)
                    if len(children) == 1:
                        parent = self.find_parent(index)
                        if (
                            parent is not None
                            and self.bt[parent] == self.bt[children[0]]
                        ):
                            # Parent and only child will be identical control nodes,
                            # child can be removed
                            up_node_index = self.find_up_node(children[0])
                            self.bt.pop(up_node_index)
                            self.bt.pop(children[0])
                    self.bt.pop(index)

    def depth(self) -> int:
        """Return depth of the bt."""
        depth = 0
        max_depth = 0

        for i in range(len(self.bt)):
            if self.behaviors.is_control_node(self.bt[i]):
                depth += 1
                max_depth = max(depth, max_depth)
            elif self.behaviors.is_up_node(self.bt[i]):
                depth -= 1
                if (depth < 0) or (depth == 0 and i is not len(self.bt) - 1):
                    return -1

        if depth != 0:
            return -1

        return max_depth

    def length(self) -> int:
        """Count number of nodes in bt. Doesn't count up characters."""
        length = 0
        for node in self.bt:
            if not self.behaviors.is_up_node(node):
                length += 1
        return length

    def random_node(self, p_leaf: float) -> ParameterizedNode:
        """Return a random node (leaf node for a given probability, otherwise control node)."""
        if random.random() > p_leaf:
            return self.behaviors.get_random_control_node()
        return self.behaviors.get_random_leaf_node()

    def change_node(self, index: int, p_leaf: float, new_node: str = None) -> None:
        """Change node at index.
        substitute old with new.
        """
        if self.behaviors.is_up_node(self.bt[index]):
            return

        # if new_node is None, change to random node
        if new_node is None:
            new_node = self.random_node(p_leaf)

        # Change control node to leaf node, remove whole subtree
        if self.behaviors.is_control_node(
            self.bt[index]
        ) and self.behaviors.is_leaf_node(new_node):
            self.delete_node(index)
            self.bt.insert(index, new_node)

        # Change leaf node to control node. Add up and extra condition/behavior node child
        elif self.behaviors.is_control_node(new_node) and self.behaviors.is_leaf_node(
            self.bt[index]
        ):
            old_node = self.bt[index]
            self.bt[index] = new_node
            if self.behaviors.is_behavior_node(old_node):
                self.bt.insert(index + 1, self.behaviors.get_random_leaf_node())
                self.bt.insert(index + 2, old_node)
            else:  # condition node
                self.bt.insert(index + 1, old_node)
                self.bt.insert(index + 2, self.behaviors.get_random_behavior_node())
            self.bt.insert(index + 3, self.behaviors.get_up_node())
        else:
            self.bt[index] = new_node

    def add_node(
        self, index: int, p_leaf: float, new_node: ParameterizedNode = None
    ) -> None:
        """Add new node at index."""

        if new_node is None:
            new_node = self.random_node(p_leaf)

        # insert control node
        if self.behaviors.is_control_node(new_node):
            if index == 0:
                # Adding new control node to encapsulate entire tree
                self.bt.insert(index, new_node)
                self.bt.append(self.behaviors.get_up_node())
            else:
                self.bt.insert(index, new_node)
                self.bt.insert(index + 1, self.behaviors.get_random_leaf_node())
                self.bt.insert(index + 2, self.behaviors.get_random_behavior_node())
                self.bt.insert(index + 3, self.behaviors.get_up_node())

        # otherwise insert leaf node
        else:
            self.bt.insert(index, new_node)

    def delete_node(self, index: int) -> None:
        """Delete node at index."""
        if self.behaviors.is_up_node(self.bt[index]):
            return

        if self.behaviors.is_control_node(self.bt[index]):
            up_node_index = self.find_up_node(index)
            for i in range(up_node_index, index, -1):
                self.bt.pop(i)

        self.bt.pop(index)

    def find_parent(self, index: int) -> None or int:
        """Return index of the CLOSEST parent to the node at input index."""
        if index == 0:
            return None

        parent = index
        siblings_left = 0
        while parent > 0:
            parent -= 1
            if self.behaviors.is_control_node(self.bt[parent]):
                if siblings_left == 0:
                    return parent
                siblings_left -= 1
            elif self.behaviors.is_up_node(self.bt[parent]):
                siblings_left += 1

        return None

    def find_children(self, index: int) -> List[int]:
        """Find ALL children to the node at index."""
        children = []
        if self.behaviors.is_control_node(self.bt[index]):
            child = index + 1
            level = 0
            while level >= 0:
                if self.behaviors.is_up_node(self.bt[child]):
                    level -= 1
                elif level == 0:
                    children.append(child)

                if self.behaviors.is_control_node(self.bt[child]):
                    level += 1
                child += 1

        return children

    def find_up_node(self, index: int) -> int:
        """Return index of the up node connected to the control node at input index.
        you MUST input the index of an up node!
        """
        if not self.behaviors.is_control_node(self.bt[index]):
            raise Exception("Invalid call. Node at index not a control node")

        if index == 0:
            if self.behaviors.is_up_node(self.bt[len(self.bt) - 1]):
                index = len(self.bt) - 1
            else:
                raise Exception("Changing invalid BT. Missing up.")
        else:
            level = 1
            while level > 0:
                index += 1
                if index == len(self.bt):
                    raise Exception("Changing invalid BT. Missing up.")
                if self.behaviors.is_control_node(self.bt[index]):
                    level += 1
                elif self.behaviors.is_up_node(self.bt[index]):
                    level -= 1

        return index

    def get_subtree(self, index: int) -> List[ParameterizedNode]:
        """Get subtree starting at index."""
        if self.behaviors.is_control_node(self.bt[index]):
            return self.bt[index : self.find_up_node(index) + 1]
        if self.behaviors.is_leaf_node(self.bt[index]):
            return [self.bt[index]]
        return []

    def insert_subtree(self, subtree: List[ParameterizedNode], index: int) -> None:
        """Insert subtree at given index."""
        self.bt[index:index] = subtree

    def swap_subtrees(self, bt2: "BT", index1: int, index2: int) -> None:
        """Swap two subtrees at given indices.
        the substrees are from self and bt2 respectively
        """
        subtree1 = self.get_subtree(index1)
        subtree2 = bt2.get_subtree(index2)

        if subtree1 != [] and subtree2 != []:
            # Remove subtrees that will be replaced
            for _ in range(len(subtree1)):
                self.bt.pop(index1)
            for _ in range(len(subtree2)):
                bt2.bt.pop(index2)

            self.insert_subtree(subtree2, index1)
            bt2.insert_subtree(subtree1, index2)

    def is_subtree(self, index: int) -> bool:
        """Check if node at index is root of a subtree."""
        return bool(
            0 <= index < len(self.bt) and not self.behaviors.is_up_node(self.bt[index])
        )

    def get_siblings(self, index: int) -> List[int]:
        """Find the siblings of the node at index."""
        siblings = []
        if self.behaviors.is_up_node(self.bt[index]):
            return siblings

        if self.behaviors.is_leaf_node(self.bt[index]):
            right = index + 1
        else:  # control node
            right = self.find_up_node(index) + 1
        while right < len(self.bt) and not self.behaviors.is_up_node(self.bt[right]):
            if self.behaviors.is_leaf_node(self.bt[right]):
                siblings.append(right)
                right += 1
            else:  # control node
                siblings.append(right)
                right = self.find_up_node(right) + 1

        left = index - 1
        while left > 0 and self.behaviors.is_leaf_node(self.bt[left]):
            siblings.append(left)
            left -= 1

        return siblings

    def swap_siblings(self, index: int) -> bool:
        """Swap position with a sibling found by get_siblings()."""
        siblings = self.get_siblings(index)
        if len(siblings) == 0:
            return False
        swap_index = random.choice(siblings)
        subtree_swap = self.get_subtree(swap_index)
        subtree_selected = self.get_subtree(index)

        if swap_index > index:
            self.delete_node(swap_index)
            self.delete_node(index)
            self.insert_subtree(subtree_swap, index)
            self.insert_subtree(
                subtree_selected, swap_index + len(subtree_swap) - len(subtree_selected)
            )
        else:
            self.delete_node(index)
            self.delete_node(swap_index)
            self.insert_subtree(subtree_selected, swap_index)
            self.insert_subtree(
                subtree_swap, index + len(subtree_selected) - len(subtree_swap)
            )

        return True

    def replace_parent_with_subtree(self, index: int) -> bool:
        """
        Replace the parent subtree with the given subtree.

        This operation might simplify the structure.
        """
        if self.behaviors.is_up_node(self.bt[index]):
            return False
        parent_index = self.find_parent(index)
        if parent_index is not None:
            m_subtree = self.get_subtree(index)
            self.delete_node(parent_index)
            self.insert_subtree(m_subtree, parent_index)
            return True

        return False

    def change_variable(self, index: int, attempt: int = 10) -> bool:
        """
        Change the variable of a given leaf node.

        The variable is not allowed to be the same as before

        fails if the node at the index is not a leaf node or
        if the node is not set to a new value after <attempt> times trials
        """
        node = self.bt[index]
        if self.behaviors.is_leaf_node(node):
            node_variable = node.get_parameters()
            while attempt > 0:
                attempt -= 1
                self.bt[index].add_random_parameters()
                if self.bt[index].get_parameters() != node_variable:
                    return True

        return False

    def get_edit_distance(
        self, second_bt: "BT", first_root: int, second_root: int, k: float = 0.5
    ) -> float or int:
        """
        Return edit distance.

        Edit distance computed as proposed in: 'Diversity in Genetic Programming: An Analysis
        of Measures and Correlation With Fitness' by Edmund K.B. et.al

        when k=1, returns un-normalized type 1 edit distance
        when k=0.5, returns type 2 edit distance

        Two trees are padded to the same shape the edit distance is the sum of two parts:
        1. root distance: if root is not equal add 1 to the distance
        2. subtree distance: the sum of subtree edit distances weighted by k
        """
        if len(second_bt.bt) > 0:
            distance = 0 if self.bt[first_root] == second_bt.bt[second_root] else 1
        else:
            distance = 1

        first_children, second_children = [], []
        if self.behaviors.is_control_node(self.bt[first_root]):
            first_children.append(first_root + 1)
            first_children += self.get_siblings(first_root + 1)
        if len(second_bt.bt) > 0 and second_bt.behaviors.is_control_node(
            second_bt.bt[second_root]
        ):
            second_children.append(second_root + 1)
            second_children += second_bt.get_siblings(second_root + 1)

        for i in range(min(len(first_children), len(second_children))):
            distance += k * self.get_edit_distance(
                second_bt,
                first_root=first_children[i],
                second_root=second_children[i],
                k=k,
            )
        empty_tree = BT([], None)
        if len(first_children) > len(second_children):
            for i in range(len(second_children), len(first_children)):
                distance += k * self.get_edit_distance(
                    empty_tree, first_root=first_children[i], second_root=-1, k=k
                )
        if len(first_children) < len(second_children):
            for i in range(len(first_children), len(second_children)):
                distance += k * second_bt.get_edit_distance(
                    empty_tree, first_root=-1, second_root=second_children[i], k=k
                )

        return distance

    def get_pseudo_isomorphs_tuple(self) -> Tuple[int, int, int]:
        """
        Return the pseudo isomorphs tuple for computing genotype diversity.

        Tuple definition: <terminals, non terminals, depth>
        """
        leaf_count = 0
        for n in self.bt:
            if self.behaviors.is_leaf_node(n):
                leaf_count += 1
        num_node = len(self.bt)
        num_control = (num_node - leaf_count) // 2
        return (leaf_count, num_control, self.depth())
