import os

# for invoke
from langchain_core.messages import HumanMessage

################################################
# * nodes
from langgraph.prebuilt import ToolInvocation
import json
from langchain_core.messages import FunctionMessage

###############################################
# * agent state
from typing import TypedDict, Annotated, Sequence
import operator
from langchain_core.messages import BaseMessage

###############################################
# * model
from langchain_openai import ChatOpenAI

###############################################
# * tools
from kios_agent.kios_tools.behaviortree_tools import (
    BehaviorTreeExecutorTool,
    BehaviorTreeSimulatorTool,
    WorldStateQueryTool,
)

# * tool executor
from langgraph.prebuilt import ToolExecutor

# * tool formatting
from langchain.tools.render import format_tool_to_openai_function

# * for search engine
from langchain_community.tools.tavily_search import TavilySearchResults


################################################

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"


# * tools

tools = [
    TavilySearchResults(max_results=1),
    BehaviorTreeExecutorTool(),  # ! lack the stw
    BehaviorTreeSimulatorTool(),
    WorldStateQueryTool(),
]

tool_executor = ToolExecutor(tools)


# * model
model = ChatOpenAI(temperature=0, streaming=True)

# * bind functions
functions = [format_tool_to_openai_function(t) for t in tools]
model = model.bind_functions(functions)


# * agent state
class AgentState(TypedDict):
    messages: Annotated[Sequence[BaseMessage], operator.add]
    #!  name the attribute name so the agent can append further messages to it.


# Define the function that determines whether to continue or not
def should_continue(state):
    messages = state["messages"]
    last_message = messages[-1]
    # If there is no function call, then we finish
    if "function_call" not in last_message.additional_kwargs:
        return "end"
    # Otherwise if there is, we continue
    else:
        return "continue"


# Define the function that calls the model
def call_model(state):
    messages = state["messages"]
    response = model.invoke(messages)
    # We return a list, because this will get added to the existing list
    return {"messages": [response]}


# Define the function to execute tools
def call_tool(state):
    messages = state["messages"]
    # Based on the continue condition
    # we know the last message involves a function call
    last_message = messages[-1]
    # We construct an ToolInvocation from the function_call
    action = ToolInvocation(
        tool=last_message.additional_kwargs["function_call"]["name"],
        tool_input=json.loads(
            last_message.additional_kwargs["function_call"]["arguments"]
        ),
    )
    # We call the tool_executor and get back a response
    response = tool_executor.invoke(action)
    # We use the response to create a FunctionMessage
    function_message = FunctionMessage(content=str(response), name=action.tool)
    # We return a list, because this will get added to the existing list
    return {"messages": [function_message]}


# * graph

from langgraph.graph import StateGraph, END

# Define a new graph
workflow = StateGraph(AgentState)

# Define the two nodes we will cycle between
workflow.add_node("agent", call_model)
workflow.add_node("action", call_tool)


# Set the entrypoint as `agent`
# This means that this node is the first one called
workflow.set_entry_point("agent")

# We now add a conditional edge
workflow.add_conditional_edges(
    # First, we define the start node. We use `agent`.
    # This means these are the edges taken after the `agent` node is called.
    "agent",
    # Next, we pass in the function that will determine which node is called next.
    should_continue,
    # Finally we pass in a mapping.
    # The keys are strings, and the values are other nodes.
    # END is a special node marking that the graph should finish.
    # What will happen is we will call `should_continue`, and then the output of that
    # will be matched against the keys in this mapping.
    # Based on which one it matches, that node will then be called.
    {
        # If `tools`, then we call the tool node.
        "continue": "action",
        # Otherwise we finish.
        "end": END,
    },
)

# We now add a normal edge from `tools` to `agent`.
# This means that after `tools` is called, `agent` node is called next.
workflow.add_edge("action", "agent")

# Finally, we compile it!
# This compiles it into a LangChain Runnable,
# meaning you can use it as you would any other runnable
app = workflow.compile()

# * run

# * streaming
inputs = {"messages": [HumanMessage(content="what is the weather in sf")]}
for output in app.stream(inputs):
    # stream() yields dictionaries with output keyed by node name
    for key, value in output.items():
        print(f"Output from node '{key}':")
        print("---")
        print(value)
    print("\n---\n")
