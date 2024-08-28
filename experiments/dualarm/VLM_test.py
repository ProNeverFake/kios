import base64
from mimetypes import guess_type
import os

from langchain_openai import ChatOpenAI
from langchain_core.prompts.chat import HumanMessagePromptTemplate, ChatPromptTemplate

# import socket, socks

# socks.set_default_proxy(socks.SOCKS5, "127.0.0.1", 1080)
# socket.socket = socks.socksocket

# os.environ["http_proxy"] = "http://127.0.0.1:7890"
# os.environ["https_proxy"] = "https://127.0.0.1:7890"

os.environ["http_proxy"] = "http://localhost:80"
# os.environ["https_proxy"] = "https://localhost:443"

# Function to encode a local image into data URL 
def local_image_to_data_url(image_path):
    mime_type, _ = guess_type(image_path)
    # Default to png
    if mime_type is None:
        mime_type = 'image/png'

    # Read and encode the image file
    with open(image_path, "rb") as image_file:
        base64_encoded_data = base64.b64encode(image_file.read()).decode('utf-8')

    # Construct the data URL
    return f"data:{mime_type};base64,{base64_encoded_data}"

prompt_template =  HumanMessagePromptTemplate.from_template(
            template=[
                {"type": "text", "text": "Summarize this image"},
                {
                    "type": "image_url",
                    "image_url": "{encoded_image_url}",
                },
            ]
        )

summarize_image_prompt = ChatPromptTemplate.from_messages([prompt_template])

model = ChatOpenAI(model= "gpt-4o"
                #    , model_kwargs={"proxies": {'http': 'http://localhost:80','https': 'http://localhost:443'}},
                #    , base_url="https://api.lingyiwanwu.com/v1"
                   )
gpt4_image_chain = summarize_image_prompt | model 

curr_dir = os.path.dirname(os.path.realpath(__file__))

img_file = os.path.join(curr_dir, "megvii.png")
image_encoded_url = local_image_to_data_url(img_file)

response = gpt4_image_chain.invoke(input={"encoded_image_url":image_encoded_url})

print(response)