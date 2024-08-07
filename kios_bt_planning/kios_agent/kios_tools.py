from typing import Optional, Type, List

"""
! not in use yet. 
this part will become necessary if you want to implement an agent.
"""

# Import things that are needed generically
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool, StructuredTool, tool

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_bt.data_types import TreeResult


class BehaviorTreeInput(BaseModel):
    json_data: dict = Field(
        description="json data for the robot task plan, including the task plan in behavior tree and the initial state of the world."
    )


class BehaviorTreeExecutorTool(BaseTool):
    name = "behavior tree executor"
    description = "useful for make the robot execute the behavior tree plan when it is verified and ready to be executed to achieve the task goal"
    args_schema: Type[BaseModel] = BehaviorTreeInput

    bt_stw: BehaviorTreeStewardship

    def __init__(self, bt_stw: BehaviorTreeStewardship):
        self.bt_stw = bt_stw
        super().__init__()

    def _run(
        self, json_data: dict, run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return "LangChain"

    async def _arun(
        self, query: str, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")


class BehaviorTreeInput(BaseModel):
    behavior_tree: dict = Field(
        description="json data for the robot task plan, including the task plan in behavior tree and the initial state of the world."
    )
    world_state: dict = Field(description="json data for the current world state.")


class BehaviorTreeSimulatorTool(BaseTool):
    name = "behavior tree simulator tool"
    description = "useful for simulating the behavior tree plan to see if the plan can achieve the task goal in the ideal case"
    args_schema: Type[BaseModel] = BehaviorTreeInput

    def _run(
        self,
        behavior_tree: dict,
        world_state: dict,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> TreeResult:
        """Use the tool."""

        bt_stw: BehaviorTreeStewardship = self.metadata["bt_stw"]
        tree_result = bt_stw.fake_run(bt_json=behavior_tree, world_state=world_state)

        return tree_result

    async def _arun(
        self,
        behavior_tree: dict,
        world_state: dict,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None,
    ) -> TreeResult:
        """Use the tool asynchronously."""

        bt_stw: BehaviorTreeStewardship = self.metadata["bt_stw"]
        tree_result = bt_stw.fake_run(bt_json=behavior_tree, world_state=world_state)

        return tree_result


class WorldStateQuery(BaseModel):
    query: str = Field(description="a string to explain the purpose of this query.")


class WorldStateQueryTool(BaseTool):
    name = "world state query tool"
    description = "useful for querying the current world state"
    args_schema: Type[BaseModel] = WorldStateQuery

    bt_stw: BehaviorTreeStewardship

    def __init__(self, bt_stw: BehaviorTreeStewardship):
        self.bt_stw = bt_stw
        super().__init__()

    def _run(
        self, query: str, run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return

    async def _arun(
        self, query: str, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")
