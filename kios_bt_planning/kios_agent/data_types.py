from dataclasses import dataclass, field
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


@dataclass
class KiosPromptSkeleton:
    """Kios prompt template"""

    skeleton_name: str
    model_name: str
    version: str

    prompt_dir: str
    prompt_load_order: List[str]

    fullname: str = field(default=None)

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "KiosPromptSkeleton":
        skeleton_name = json["skeleton_name"]
        prompt_dir = json["prompt_dir"]
        prompt_load_order = json["prompt_load_order"]
        version = json["version"] if "version" in json else "VX"
        model_name = json["model_name"] if "model_name" in json else "gptX"
        fullname = f"{skeleton_name}_{version}_{model_name}"
        return KiosPromptSkeleton(
            skeleton_name=skeleton_name,
            model_name=model_name,
            version=version,
            prompt_dir=prompt_dir,
            prompt_load_order=prompt_load_order,
            fullname=fullname,
        )
