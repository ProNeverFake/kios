"""
extracting citations, showing which from the source document are used for
generating the answer and which are not.

"which parts are truly in use?"

this is quite critical when retrieving from online sources like wiki, where
the retrieved snippets are not always used for generating the answer.

"""

# ! skipped Direct prompting which is a hand-crafted citation example

# * or we can do post processing after retrieving the sources,
# * to select the most relevant snippets to use for generating the answer.
# * this asks for another splitting, embedding and retrieval,
# * which is not efficient but can save tokens.


# * Another approach is to post-process our model generation.
# * get answer first, and ask the model to annotate the sources used.
# ! this will cost a lot of tokens!!!

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

from langchain_community.retrievers import WikipediaRetriever
from langchain_core.prompts import ChatPromptTemplate

llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0)
wiki = WikipediaRetriever(top_k_results=6, doc_content_chars_max=2000)
prompt = ChatPromptTemplate.from_messages(
    [
        (
            "system",
            "You're a helpful AI assistant. Given a user question and some Wikipedia article snippets, answer the user question. If none of the articles answer the question, just say you don't know.\n\nHere are the Wikipedia articles:{context}",
        ),
        ("human", "{question}"),
    ]
)

# prompt.pretty_print()

from operator import itemgetter
from typing import List

from langchain_core.documents import Document
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import (
    RunnableLambda,
    RunnableParallel,
    RunnablePassthrough,
)


def format_docs(docs: List[Document]) -> str:
    """Convert Documents to a single string.:"""
    formatted = [
        f"Article Title: {doc.metadata['title']}\nArticle Snippet: {doc.page_content}"
        for doc in docs
    ]
    return "\n\n" + "\n\n".join(formatted)


# * callable operator to extract the docs and format them
format = itemgetter("docs") | RunnableLambda(format_docs)
# * subchain for generating an answer once we've done retrieval
answer = prompt | llm | StrOutputParser()
# complete chain that calls wiki -> formats docs to string -> runs answer subchain -> returns just the answer and retrieved docs.
chain = (
    RunnableParallel(question=RunnablePassthrough(), docs=wiki)
    .assign(context=format)  # * assign chains to keys
    .assign(answer=answer)  # * assign answer chain to the answer key
    .pick(["answer", "docs"])  # * pick only the answer and the docs as the outputs
)

# chain.invoke("How fast are cheetahs?")

########################
# * use function-calling

from langchain_core.pydantic_v1 import BaseModel, Field


class cited_answer(BaseModel):
    """Answer the user question based only on the given sources, and cite the sources used."""

    answer: str = Field(
        ...,
        description="The answer to the user question, which is based only on the given sources.",
    )
    citations: List[int] = Field(
        ...,
        description="The integer IDs of the SPECIFIC sources which justify the answer.",
    )


# * use tool "class"
llm_with_tool = llm.bind_tools(
    [cited_answer],
    tool_choice="cited_answer",
)
example_q = """What Brian's height?

Source: 1
Information: Suzy is 6'2"

Source: 2
Information: Jeremiah is blonde

Source: 3
Information: Brian is 3 inches shorted than Suzy"""

# llm_with_tool.invoke(example_q)  # * default is "question"


# * output parser to extract the cited answer
from langchain.output_parsers.openai_tools import JsonOutputKeyToolsParser

output_parser = JsonOutputKeyToolsParser(key_name="cited_answer", return_single=True)
# (llm_with_tool | output_parser).invoke(example_q)


def format_docs_with_id(docs: List[Document]) -> str:
    """
    get the source id, article title and article snippet of the documents
    return as a string
    """
    formatted = [
        f"Source ID: {i}\nArticle Title: {doc.metadata['title']}\nArticle Snippet: {doc.page_content}"
        for i, doc in enumerate(docs)
    ]
    return "\n\n" + "\n\n".join(formatted)


# format chain
format_1 = itemgetter("docs") | RunnableLambda(format_docs_with_id)
# answer chain
answer_1 = prompt | llm_with_tool | output_parser
# the full chain
chain_1 = (
    RunnableParallel(question=RunnablePassthrough(), docs=wiki)
    .assign(context=format_1)
    .assign(cited_answer=answer_1)
    .pick(["cited_answer", "docs"])
)

# for chunk in chain_1.stream("How fast are cheetahs?"):
#     print(chunk, end="", flush=True)

# * cite snippets


class Citation(BaseModel):
    source_id: int = Field(
        ...,
        description="The integer ID of a SPECIFIC source which justifies the answer.",
    )
    quote: str = Field(
        ...,
        description="The VERBATIM quote from the specified source that justifies the answer.",
    )


class quoted_answer(BaseModel):
    """Answer the user question based only on the given sources, and cite the sources used."""

    answer: str = Field(
        ...,
        description="The answer to the user question, which is based only on the given sources.",
    )
    citations: List[Citation] = Field(
        ..., description="Citations from the given sources that justify the answer."
    )


output_parser_2 = JsonOutputKeyToolsParser(key_name="quoted_answer", return_single=True)
llm_with_tool_2 = llm.bind_tools(
    [quoted_answer],
    tool_choice="quoted_answer",
)
format_2 = itemgetter("docs") | RunnableLambda(format_docs_with_id)
answer_2 = prompt | llm_with_tool_2 | output_parser_2
chain_2 = (
    RunnableParallel(question=RunnablePassthrough(), docs=wiki)
    .assign(context=format_2)
    .assign(quoted_answer=answer_2)
    .pick(["quoted_answer", "docs"])
)

# for chunk in chain_2.stream("How fast are cheetahs?"):
#     print(chunk, end="", flush=True)

# * post processing

from langchain.retrievers.document_compressors import EmbeddingsFilter
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_openai import OpenAIEmbeddings

splitter = RecursiveCharacterTextSplitter(
    chunk_size=400,
    chunk_overlap=0,
    separators=["\n\n", "\n", ".", " "],
    keep_separator=False,
)
compressor = EmbeddingsFilter(embeddings=OpenAIEmbeddings(), k=10)


def split_and_filter(input) -> List[Document]:
    docs = input["docs"]
    question = input["question"]
    split_docs = splitter.split_documents(docs)
    stateful_docs = compressor.compress_documents(split_docs, question)
    return [stateful_doc for stateful_doc in stateful_docs]


retrieve = (
    RunnableParallel(question=RunnablePassthrough(), docs=wiki) | split_and_filter
)
docs = retrieve.invoke("How fast are cheetahs?")
# for doc in docs:
#     print(doc.page_content)
#     print("\n\n")

chain_4 = (
    RunnableParallel(question=RunnablePassthrough(), docs=retrieve)
    .assign(context=format)
    .assign(answer=answer)
    .pick(["answer", "docs"])
)

# Note the documents have an article "summary" in the metadata that is now much longer than the
# actual document page content. This summary isn't actually passed to the model.
# chain_4.invoke("How fast are cheetahs?")

# * Generation post-processing


class Citation(BaseModel):
    source_id: int = Field(
        ...,
        description="The integer ID of a SPECIFIC source which justifies the answer.",
    )
    quote: str = Field(
        ...,
        description="The VERBATIM quote from the specified source that justifies the answer.",
    )


class annotated_answer(BaseModel):
    """Annotate the answer to the user question with quote citations that justify the answer."""

    citations: List[Citation] = Field(
        ..., description="Citations from the given sources that justify the answer."
    )


llm_with_tools_5 = llm.bind_tools(
    [annotated_answer],
    tool_choice="annotated_answer",
)

from langchain_core.prompts import MessagesPlaceholder

prompt_5 = ChatPromptTemplate.from_messages(
    [
        (
            "system",
            "You're a helpful AI assistant. Given a user question and some Wikipedia article snippets, answer the user question. If none of the articles answer the question, just say you don't know.\n\nHere are the Wikipedia articles:{context}",
        ),
        ("human", "{question}"),
        MessagesPlaceholder("chat_history", optional=True),  # * optional chat history
    ]
)
answer_5 = prompt_5 | llm  # * answer chain needs llm
annotation_chain = (
    prompt_5
    | llm_with_tools_5
    | JsonOutputKeyToolsParser(key_name="annotated_answer", return_single=True)
    | itemgetter("citations")
)
# * the assign are executed one by one, you can check it in langsmith monitor.
chain_5 = (
    RunnableParallel(question=RunnablePassthrough(), docs=wiki)
    .assign(
        context=format
    )  # * retrieve the sources and format them, assign to the context key
    .assign(
        ai_message=answer_5
    )  # * generate the answer and annotate the sources used, assign to the ai_message key
    .assign(
        chat_history=(lambda x: [x["ai_message"]]),
        answer=(lambda x: x["ai_message"].content),
    )  # * assign the chat history and the answer to the chat_history and answer keys
    .assign(
        annotations=annotation_chain
    )  # * annotate the sources used, assign to the annotations key
    .pick(
        ["answer", "docs", "annotations"]
    )  # * pick the answer, the sources and the annotations as the outputs
)

chain_5.invoke("How fast are cheetahs?")
