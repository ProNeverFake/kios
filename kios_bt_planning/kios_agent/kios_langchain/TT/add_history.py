"""
the application needs some sort of “memory” of past questions and answers, 
and some logic for incorporating those into its current thinking.

use a chain to handle the chat history and the latest user question to
reformulate the question to be standalone.
then feed the standalone question to the rag_chain to get the answer.

not the true memory.
need reroutine.

"""

import bs4
from langchain import hub
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import WebBaseLoader
from langchain_community.vectorstores import Chroma
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough
from langchain_openai import ChatOpenAI, OpenAIEmbeddings

import os

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"

# Load, chunk and index the contents of the blog.
loader = WebBaseLoader(
    web_paths=("https://lilianweng.github.io/posts/2023-06-23-agent/",),
    bs_kwargs=dict(
        parse_only=bs4.SoupStrainer(
            class_=("post-content", "post-title", "post-header")
        )
    ),
)
docs = loader.load()

text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
splits = text_splitter.split_documents(docs)
vectorstore = Chroma.from_documents(documents=splits, embedding=OpenAIEmbeddings())

# Retrieve and generate using the relevant snippets of the blog.
retriever = vectorstore.as_retriever()
prompt = hub.pull("rlm/rag-prompt")
llm = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0)


def format_docs(docs):
    return "\n\n".join(doc.page_content for doc in docs)


# rag_chain = (
#     {"context": retriever | format_docs, "question": RunnablePassthrough()}
#     | prompt
#     | llm
#     | StrOutputParser()
# )

"""
 a sub-chain that takes historical messages and the latest user question,
 and reformulates the question to be standalone
"""

from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

contextualize_q_system_prompt = """Given a chat history and the latest user question \
which might reference context in the chat history, formulate a standalone question \
which can be understood without the chat history. Do NOT answer the question, \
just reformulate it if needed and otherwise return it as is."""

# * the prompt template to make a standalone question
contextualize_q_prompt = ChatPromptTemplate.from_messages(
    [
        ("system", contextualize_q_system_prompt),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{question}"),
    ]
)
contextualize_q_chain = contextualize_q_prompt | llm | StrOutputParser()

""" Example usage
"""

from langchain_core.messages import AIMessage, HumanMessage

# the context chain to make a standalone question
contextualize_q_chain.invoke(  # * invoke needs to fill out the missing part of the prompt.
    {
        # * so this part is a hand-crafted example of a chat history and a question
        "chat_history": [
            HumanMessage(content="What does LLM stand for?"),
            AIMessage(content="Large language model"),  # answered by the user
        ],
        "question": "What is meant by large",
    }
)

# * the full chain

qa_system_prompt = """You are an assistant for question-answering tasks. \
Use the following pieces of retrieved context to answer the question. \
If you don't know the answer, just say that you don't know. \
Use three sentences maximum and keep the answer concise.\

{context}"""

# * the template for the prompt
qa_prompt = ChatPromptTemplate.from_messages(
    [
        ("system", qa_system_prompt),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{question}"),
    ]
)


"""
reroutine the rag chain.
if the input has a chat history, it will use the contextualize_q_chain to make a standalone question.
if not, the question should be the standalone one. it will use the question as is.
"""


def contextualized_question(input: dict):
    if input.get(
        "chat_history"
    ):  # * In Python, an empty list [] is considered False in a boolean context.
        return (
            contextualize_q_chain  # in which you need the chat history and the question
        )
    else:
        return input["question"]


# * the chain
rag_chain = (
    RunnablePassthrough.assign(
        context=contextualized_question | retriever | format_docs
    )
    | qa_prompt
    | llm
)

chat_history = []

# * the first question
question = "What is Task Decomposition?"
ai_msg = rag_chain.invoke({"question": question, "chat_history": chat_history})
chat_history.extend([HumanMessage(content=question), ai_msg])

# * the second question
second_question = "What are common ways of doing it?"
rag_chain.invoke({"question": second_question, "chat_history": chat_history})
