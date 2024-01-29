from dataclasses import dataclass
from typing import List, Optional, Dict, Any


@dataclass
class AgentResponse:
    """Response from llm agent"""

    action_sequence: List[str]
    ...

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "AgentResponse":
        return AgentResponse(
            action_sequence=json["action_sequence"],
        )
