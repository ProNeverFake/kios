from typing import Any
import py_trees


class WorldInterface:
    def __init__(self) -> None:
        self.blackboard = py_trees.blackboard.Client(name=self.__class__.__name__)
        pass

    def get_object(self, object_name: str) -> Any:
        raise NotImplementedError

    def get_objects(self, object_name: str) -> Any:
        raise NotImplementedError

    def set_state(self, state: dict) -> None:
        raise NotImplementedError

    def get_state(self) -> dict:
        raise NotImplementedError

    def query_state(self, query: dict) -> bool:
        raise NotImplementedError
