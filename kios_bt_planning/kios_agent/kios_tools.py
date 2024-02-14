# Import things that are needed generically
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool, StructuredTool, tool

from typing import Optional, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)


# * tool decorator
@tool
def search(query: str) -> str:
    """Look up things online."""
    return "LangChain"


# print(search.name)
# print(search.description)
# print(search.args)


@tool
def multiply(a: int, b: int) -> int:
    """Multiply two numbers."""
    return a * b


# print(multiply.name)
# print(multiply.description)
# print(multiply.args)


class SearchInput(BaseModel):
    query: str = Field(description="should be a search query")


@tool("search-tool", args_schema=SearchInput, return_direct=True)
def search(query: str) -> str:
    """Look up things online."""
    return "LangChain"


# print(search.name)
# print(search.description)
# print(search.args)
# print(search.return_direct)


# * BaseTool
class SearchInput(BaseModel):
    query: str = Field(description="should be a search query")


class CalculatorInput(BaseModel):
    a: int = Field(description="first number")
    b: int = Field(description="second number")


class CustomSearchTool(BaseTool):
    name = "custom_search"
    description = "useful for when you need to answer questions about current events"
    args_schema: Type[BaseModel] = SearchInput

    def _run(
        self, query: str, run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return "LangChain"

    async def _arun(
        self, query: str, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")


class CustomCalculatorTool(BaseTool):
    name = "Calculator"
    description = "useful for when you need to answer questions about math"
    args_schema: Type[BaseModel] = CalculatorInput
    return_direct: bool = True

    def _run(
        self, a: int, b: int, run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return a * b

    async def _arun(
        self,
        a: int,
        b: int,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None,
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Calculator does not support async")


# search = CustomSearchTool()
# print(search.name)
# print(search.description)
# print(search.args)

# multiply = CustomCalculatorTool()
# print(multiply.name)
# print(multiply.description)
# print(multiply.args)
# print(multiply.return_direct)


# * StructuredTool
def search_function(query: str):
    return "LangChain"


search = StructuredTool.from_function(
    func=search_function,
    name="Search",
    args_schema=SearchInput,
    description="useful for when you need to answer questions about current events",
    # * coroutine= ... <- you can specify an async method if desired as well
)

# print(search.name)
# print(search.description)
# print(search.args)

# * handling errors
from langchain_core.tools import ToolException


def search_tool1(s: str):
    raise ToolException("The search tool1 is not available.")


search = StructuredTool.from_function(
    func=search_tool1,
    name="Search_tool1",
    description="A bad tool",
    handle_tool_error=True,
)

# search.run("test")


def _handle_error(error: ToolException) -> str:
    return (
        "The following errors occurred during tool execution:"
        + error.args[0]
        + "Please try another tool."
    )


search = StructuredTool.from_function(
    func=search_tool1,
    name="Search_tool1",
    description="A bad tool",
    handle_tool_error=_handle_error,
)

# print(search.run("test"))
# search.run("test")
