from typing import Dict, List, Any
from kios_robot.data_types import MiosObject


class ObjectMap:
    object_map: Dict[str, MiosObject]

    def __init__(self):
        self.object_map = {}
