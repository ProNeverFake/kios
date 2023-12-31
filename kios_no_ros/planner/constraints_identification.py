"""The constraints module contains functions for inferring task constraints.

- for LFD.

"""

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

import logging
from typing import Any, Dict, List, Type


logger = logging.getLogger("constraints_identification")


class Constraint:
    """
    for LFD.
    A single ordering constraint of two actions.
    """

    def __init__(self, action_a, action_b, distance: float = 1):
        """
        Create a constraint action_a < action_b.

        Both actions must be instances of EquivalentAction.
        """
        if action_a is action_b:
            raise ValueError("Both actions are the same object.")

        self.first = action_a
        self.second = action_b
        self.distance = distance

    def reverse_constraint(self) -> Type["Constraint"]:
        """
        Reverse the logic order of the constraint.

        If this constraint is A < B, it returns a new Constraint representing B < A.
        """
        return Constraint(self.second, self.first)

    def __eq__(self, other: Type["Constraint"]) -> bool:
        """Return True if the constraint is equal to other."""
        return self.first == other.first and self.second == other.second

    def __str__(self) -> str:
        """Print the constraints to string."""
        return f"{self.first} < {self.second}"


class ConstraintSet:
    """
    For LFD.
    A set of constraints."""

    def __init__(self):
        """Construct a new empty set of constraints."""
        self.__active = []
        self.__conflicting = []

    def active_constraints(self) -> List["Constraint"]:
        """Return a list of active constraints that do not conflict with each other."""
        return self.__active

    def conflicting_constraints(self) -> List["Constraint"]:
        """Return a list of conflicting constraints."""
        return self.__conflicting

    def add(self, constraint: Type["Constraint"]):
        """
        Add a new constraint.

        If it conflicts with an active constraint, the active constraint will be removed
        and added to conflicting constraints. The new constraint is ignored.

        Args:
        ----
            constraint: the new constraint to add.

        """
        if constraint in self.__conflicting:
            return

        if constraint.reverse_constraint() in self.__active:
            # The constraint conflicts with an active constraint
            self.__active.remove(constraint.reverse_constraint())
            self.__conflicting.append(constraint)
            self.__conflicting.append(constraint.reverse_constraint())
        elif constraint in self.__active:
            # If the constraint has been encountered previously, we update to the new constraint
            # if it has less distance
            constraint_idx = self.__active.index(constraint)
            if constraint.distance < self.__active[constraint_idx].distance:
                del self.__active[constraint_idx]
                self.__active.append(constraint)
        else:
            self.__active.append(constraint)


def infer_order_constraints(demonstrations: List) -> Type["ConstraintSet"]:
    """
    for LFD.
    Infer the order of the constraints in the demonstration.

    Args
    ----
        demonstrations: list of demos where each demon is a list of EquivalentAction.

    Returns
    -------
        A ConstraintSet of order constraints.

    """
    constraints = ConstraintSet()

    for demonstration in demonstrations:
        for action_a_idx, action_a in enumerate(demonstration):
            if action_a_idx >= len(demonstration) - 1:
                # We have reached the end of demonstration. There is no next action.
                continue

            for action_b_idx in range(action_a_idx + 1, len(demonstration)):
                if not demonstration[action_b_idx].can_form_constraint():
                    continue
                if action_a is not demonstration[action_b_idx]:
                    # Do not add constraints between an action and itself.
                    constraints.add(
                        Constraint(
                            action_a,
                            demonstration[action_b_idx],
                            action_b_idx - action_a_idx,
                        )
                    )

    msg = "Detected constraints:"
    for c in constraints.active_constraints():
        msg += "\n\t" + str(c)
    logger.info(msg)

    return constraints


def infer_preconditions(demonstrations: List, behaviors: Type[Any]):
    """
    Add additional inferred preconditions to the actions in demonstrations.

    Args:
    ----
        demonstrations: list of demos where each demo is a list of EquivalentAction.
        behaviors: instance of Behaviors.

    """
    order_constraints = infer_order_constraints(demonstrations)

    # added_conditions is a dictionary with constraints of the added conditions.
    # The first key is an EquivalentAction instance and the second key is a condition string.
    added_conditions = {}

    # Convert order constraints to preconditions
    for constraint in order_constraints.active_constraints():
        logger.debug("Handling %s", constraint)

        for condition in constraint.first.postconditions():
            # Does condition conflict with a postcondition?
            conflicting = get_conflicting(
                behaviors, constraint.second.postconditions(), condition
            )
            if conflicting is not None:
                logger.debug(
                    '"%s" conflicting with the postcondition %s', condition, conflicting
                )
                continue

            # Does the new condition conflict with an existing precondition?
            if contains_conflicting(
                behaviors, constraint.second.preconditions(), condition
            ):
                logger.debug(
                    "%s conflicts with existing precondition %s",
                    condition,
                    get_conflicting(
                        behaviors, constraint.second.preconditions(), condition
                    ),
                )
                continue

            # Does the new condition conflict with a previously added precondition?
            if contains_conflicting(
                behaviors, constraint.second.additional_preconditions, condition
            ):
                # Keep the constraint with lowest distance
                conflicting_conditions = [
                    c
                    for c in constraint.second.additional_preconditions
                    if not behaviors.compatible(c, condition)
                ]
                # conflicting_distance = lambda c: added_conditions[constraint.second][c].distance
                if all(
                    map(
                        lambda c: constraint.distance
                        < get_conflicting_distance(  # pylint: disable=cell-var-from-loop
                            added_conditions, constraint, c
                        ),  # pylint: disable=cell-var-from-loop
                        conflicting_conditions,
                    )
                ):
                    # New constraint has shorter distance than all conflicting constraints
                    for conflicting_condition in conflicting_conditions:
                        logger.debug(
                            'Replacing "%s" with "%s" for "%s"',
                            conflicting_condition,
                            condition,
                            constraint.second.action_string(),
                        )
                        constraint.second.additional_preconditions.remove(
                            conflicting_condition
                        )
                    constraint.second.additional_preconditions.append(condition)
                    added_conditions[constraint.second][condition] = constraint
                continue

            # No conflicts detected. Ok to add the new precondition if it doesn't already exist
            if condition not in constraint.second.preconditions_with_additional():
                logger.info(
                    'Added "%s" as a precondition of "%s"',
                    condition,
                    constraint.second.action_string(),
                )
                constraint.second.additional_preconditions.append(condition)
                if constraint.second not in added_conditions:
                    added_conditions[constraint.second] = {}
                added_conditions[constraint.second][condition] = constraint

            elif (
                condition not in constraint.second.preconditions()
                and constraint.distance
                < added_conditions[constraint.second][condition].distance
            ):
                # We have encountered the same condition with lower constraint distance.
                # Update the number.
                added_conditions[constraint.second][condition] = constraint


def get_conflicting_distance(
    added_conditions: Dict, constraint: Type["Constraint"], conflicting_condition: Any
) -> float:
    """
    for LFD.
    Get the distance of two conflicting conditions.

    Args
    ----
        added_conditions: dictionary with the conditions.
        constraint: constraint with conflicting conditions.
        conflicting_condition: the condition that is causing conflict.

    Returns
    -------
        The conflicting distance.

    """
    return added_conditions[constraint.second][conflicting_condition].distance


def get_conflicting(
    behaviors: Type[Any], condition_list: List[str], condition: str
) -> str or None:
    """
    Get the conflicting conditions.

    Args
    ----
        behaviors: the behavior to consider the conditions of.
        condition_list: the list of condition to check for compatibility.
        condition: baseline for comparison.

    Returns
    -------
        The first condition in condition_list that the input condition conflicts with.
        None is returned if there are no conflicts.

    """
    for c in condition_list:
        if not behaviors.compatible(condition, c):
            return c

    return None


def contains_conflicting(
    behaviors: Type[Any], condition_list: List[str], condition: str
) -> bool:
    """
    Check if the condition list contains conflicting conditions.

    Args
    ----
        behaviors: the behavior to consider the conditions of.
        condition_list: the list of condition to check for compatibility.
        condition: baseline for comparison.

    Returns
    -------
        True if condition_list contains at least one element that conflicts with input condition.

    """
    return get_conflicting(behaviors, condition_list, condition) is not None
