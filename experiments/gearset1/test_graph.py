import os
from typing import List, Tuple, Annotated, TypedDict
import operator

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"

from langgraph.graph import StateGraph, END

from langsmith import traceable


class PlanExecuteState(TypedDict):
    plan: List[str]
    behavior_tree_skeleton: dict  # ! not sure if use this or not.
    behavior_tree: dict
    world_state: Annotated[
        List[dict], operator.add
    ]  # ! to add, you need to make world_state a list of dict
    # = Field(default=None)
    past_steps: Annotated[List[Tuple], operator.add]
    response: str  # ! not sure if use this or not.
    # to_user: bool
    user_input: str
    problem: str


@traceable(name="user_input_node_step")
async def user_input_step(state: PlanExecuteState):
    user_input = input("Enter your next step: ")
    return {
        "user_input": user_input,
    }


@traceable(name="user_input_conditional_edge")
def user_input_should_continue(state: PlanExecuteState):

    if state["user_input"] == "gogogo":
        return True
    else:
        return False


workflow = StateGraph(PlanExecuteState)

workflow.add_node("user_input_node", user_input_step)

workflow.set_entry_point("user_input_node")

workflow.add_conditional_edges(
    "user_input_node",
    user_input_should_continue,
    {
        True: "user_input_node",
        False: END,
    },
)

app = workflow.compile()


async def core_run():
    async for event in app.astream(input={}):
        for k, v in event.items():
            if k != "__end__":
                print(v)


if __name__ == "__main__":
    import asyncio

    asyncio.run(core_run())
