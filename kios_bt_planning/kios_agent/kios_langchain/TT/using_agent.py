# # * get the documents
# from langchain_community.document_loaders import TextLoader

# loader = TextLoader("../../modules/state_of_the_union.txt")
# documents = loader.load()
# ! where is this file?

# # * create_retriever
# from langchain.text_splitter import CharacterTextSplitter
# from langchain_community.vectorstores import FAISS
# from langchain_openai import OpenAIEmbeddings

# text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
# texts = text_splitter.split_documents(documents)
# embeddings = OpenAIEmbeddings()
# db = FAISS.from_documents(texts, embeddings)

# retriever = db.as_retriever()

# # * create_retriever_tool
# from langchain.tools.retriever import create_retriever_tool

# tool = create_retriever_tool(
#     retriever,
#     "search_state_of_union",
#     "Searches and returns excerpts from the 2022 State of the Union.",
# )
# tools = [tool]

from langchain import hub

prompt = hub.pull("hwchase17/openai-tools-agent")
print(prompt.messages)
# prompt.messages
