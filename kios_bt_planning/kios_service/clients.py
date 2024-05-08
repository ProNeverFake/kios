from langchain.schema import SystemMessage, HumanMessage
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableMap
from langserve import RemoteRunnable

openai = RemoteRunnable("http://localhost:8000/openai/")
anthropic = RemoteRunnable("http://localhost:8000/anthropic/")
joke_chain = RemoteRunnable("http://localhost:8000/joke/")

joke_chain.invoke({"topic": "parrots"})

# or async
await joke_chain.ainvoke({"topic": "parrots"})

prompt = [
    SystemMessage(content="Act like either a cat or a parrot."),
    HumanMessage(content="Hello!"),
]

# Supports astream
async for msg in anthropic.astream(prompt):
    print(msg, end="", flush=True)

prompt = ChatPromptTemplate.from_messages(
    [("system", "Tell me a long story about {topic}")]
)

# Can define custom chains
chain = prompt | RunnableMap(
    {
        "openai": openai,
        "anthropic": anthropic,
    }
)

chain.batch([{"topic": "parrots"}, {"topic": "cats"}])
