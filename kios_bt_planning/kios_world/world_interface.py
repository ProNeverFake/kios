from typing import Any
import py_trees


class WorldInterface:
    def __init__(self) -> None:
        self.blackboard = py_trees.blackboard.Client(name=self.__class__.__name__)
        pass

    def register_predicates(self, predicates: dict) -> None:
        """
        add this predicate to the world. do nothing if the predicate already exists
        """
        if predicates is not None:
            for key, _ in predicates.items():
                self.blackboard.register_key(
                    key=key, access=py_trees.common.Access.WRITE
                )

    def get_object(self, object_name: str) -> Any:
        raise NotImplementedError

    def get_objects(self, object_name: str) -> Any:
        raise NotImplementedError

    def set_predicates(self, predicates: dict) -> None:
        """
        "take effects"
        """
        if predicates is not None:
            try:
                for key, value in predicates.items():
                    self.blackboard.set(name=key, value=value)
            except KeyError:
                print("KeyError: %s triggered this problem!" % (key))

    def get_predicates(self) -> dict:
        raise NotImplementedError

    def query_state(self, query: dict) -> bool:
        raise NotImplementedError
