import json
from typing import Any


class KiosAgent:
    llm_model_name: str

    def __init__(self, llm_model_name: str = "gpt4"):
        self.llm_model_name = llm_model_name

    def initialize(self):
        pass

    def setup_model(self):
        pass

    def model_query(self) -> json:
        pass

    def response_parse(self, response: json) -> Any:
        pass
