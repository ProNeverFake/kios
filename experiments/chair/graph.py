from langchain_core.pydantic_v1 import BaseModel, Field
from typing import List, Tuple, Annotated, TypedDict
import operator

from langchain_openai import ChatOpenAI

import os

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"

##################################################### * tool
from langchain_community.tools.tavily_search import TavilySearchResults
from kios_agent.kios_tools import BehaviorTreeExecutorTool, BehaviorTreeSimulatorTool

# tools = [TavilySearchResults(max_results=3)]
tools = [BehaviorTreeSimulatorTool()]  # ! need stw

##################################################### * execution

from langchain import hub
from langchain.agents import create_openai_functions_agent
from langchain_openai import ChatOpenAI

prompt = hub.pull("hwchase17/openai-functions-agent")

# Choose the LLM that will drive the agent
llm = ChatOpenAI(model="gpt-4-turbo-preview")
# Construct the OpenAI Functions agent
agent_runnable = create_openai_functions_agent(llm, tools, prompt)

from langgraph.prebuilt import create_agent_executor

agent_executor = create_agent_executor(agent_runnable, tools)


# * the graph state
class PlanExecuteState(TypedDict):
    input: str
    plan: List[str]
    world_state: List[dict]
    past_steps: Annotated[List[Tuple], operator.add]
    response: str


##################################################### * planner
# * output schema of the planner
class Plan(BaseModel):
    """Plan to follow in future"""

    steps: List[str] = Field(
        description="different steps to follow, should be in sorted order"
    )


from langchain.chains.openai_functions import create_structured_output_runnable
from langchain_core.prompts import ChatPromptTemplate

planner_prompt = ChatPromptTemplate.from_template(
    """For the given objective, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

{objective}"""
)
planner = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_prompt
)

##################################################### * plan_updaterner
from langchain.chains.openai_functions import create_openai_fn_runnable


class Response(BaseModel):
    """Response to user."""

    response: str


plan_updater_prompt = ChatPromptTemplate.from_template(
    """For the given objective, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

Your objective was this:
{input}

Your original plan was this:
{plan}

You have currently done the follow steps:
{past_steps}

Update your plan accordingly. If no more steps are needed and you can return to the user, then respond with that. Otherwise, fill out the plan. Only add steps to the plan that still NEED to be done. Do not return previously done steps as part of the plan."""
)

plan_updater = create_openai_fn_runnable(
    [Plan, Response],  # * here two schemas are used
    ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
    plan_updater_prompt,
)


##################################################### * graph
async def execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    task = state["plan"][0]
    agent_response = await agent_executor.ainvoke({"input": task, "chat_history": []})
    return {
        "past_steps": (task, agent_response["agent_outcome"].return_values["output"])
    }


async def plan_step(state: PlanExecuteState):
    """
    plan the steps based on user input
    """
    plan = await planner.ainvoke({"objective": state["input"]})
    return {"plan": plan.steps}


async def plan_updater_step(state: PlanExecuteState):
    """
    if return a response, then success, response the use, end.
    otherwise, return the updated newplan (normally the same as the old plan with the first step popped out.
    """
    output = await plan_updater.ainvoke(state)
    if isinstance(output, Response):  # * determine if it is time to response and end
        return {"response": output.response}
    else:
        return {"plan": output.steps}


def should_end(state: PlanExecuteState):
    """
    end router
    """
    if state["response"]:
        return True
    else:
        return False


##################################################### * construct the graph
from langgraph.graph import StateGraph, END

workflow = StateGraph(PlanExecuteState)

# Add the plan node
workflow.add_node("planner", plan_step)

# Add the execution step
workflow.add_node("agent", execute_step)

# Add a plan_updater node
workflow.add_node("plan_updater", plan_updater_step)

workflow.set_entry_point("planner")

# From plan we go to agent
workflow.add_edge("planner", "agent")

# From agent, we plan_updater
workflow.add_edge("agent", "plan_updater")

workflow.add_conditional_edges(
    "plan_updater",
    # Next, we pass in the function that will determine which node is called next.
    should_end,
    {
        # If `tools`, then we call the tool node.
        True: END,
        False: "agent",
    },
)

# compiles it into a LangChain Runnable,
app = workflow.compile()
