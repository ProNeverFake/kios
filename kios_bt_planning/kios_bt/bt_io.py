from kios_bt.data_types import Action, Condition, ObjectProperty

from typing import List, Dict, Any


class TestClass:
    roster = {}

    def from_json_to_AC(self, json_data: dict):
        """
        generate a roster from a json data
        """
        for key, value in json_data.items():
            self.roster[key] = value
