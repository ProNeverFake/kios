"""
selector without memory stolen from dynamic bt package
"""
# ! not in use yet.
# import py_trees as pt
# from typing import Any, List, Tuple


# class RSequence(pt.composites.Selector):
#     """
#     Rsequence for py_trees.

#     Reactive sequence overriding sequence with memory, py_trees' only available sequence.
#     Author: Christopher Iliffe Sprague, sprague@kth.se
#     """

#     def __init__(self, name: str = "Sequence", children: List[Any] = None):
#         super().__init__(name=name, children=children)

#     def tick(self):
#         """
#         Run the tick behaviour for this selector.

#         Note that the status of the tick is always determined by its children,
#         not by the user customized update function.

#         Yields
#         ------
#             class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children.

#         """
#         self.logger.debug(
#             "%s.tick()" % self.__class__.__name__
#         )  # pylint: disable=consider-using-f-string
#         # Required behaviour for *all* behaviours and composites is
#         # for tick() to check if it isn't running and initialise
#         if self.status != pt.common.Status.RUNNING:
#             # selectors dont do anything specific on initialization
#             #   - the current child is managed by the update, never needs to be 'initialized'
#             # run subclass (user) handles
#             self.initialise()
#         # run any work designated by a customized instance of this class
#         self.update()
#         previous = self.current_child
#         for child in self.children:
#             for node in child.tick():
#                 yield node
#                 if node is child and (
#                     node.status == pt.common.Status.RUNNING
#                     or node.status == pt.common.Status.FAILURE
#                 ):
#                     self.current_child = child
#                     self.status = node.status
#                     if previous is None or previous != self.current_child:
#                         # we interrupted, invalidate everything at a lower priority
#                         passed = False
#                         for sibling in self.children:
#                             if passed and sibling.status != pt.common.Status.INVALID:
#                                 sibling.stop(pt.common.Status.INVALID)
#                             if sibling == self.current_child:
#                                 passed = True
#                     yield self
#                     return
#         # all children succeeded,
#         # set succeed ourselves and current child to the last bugger who failed us
#         self.status = pt.common.Status.SUCCESS
#         try:
#             self.current_child = self.children[-1]
#         except IndexError:
#             self.current_child = None
#         yield self
