import base64
import io
import os

import numpy as np
from IPython.display import HTML, display
from PIL import Image

# os.environ["http_proxy"] = "http://127.0.0.1:80"
# os.environ["https_proxy"] = "https://127.0.0.1:443"

zero_one_api = os.environ.get("ZERO_ONE_API")

print(zero_one_api)

def encode_image(image_path):
    """获取图像的 base64 字符串"""

    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

def plt_img_base64(img_base64):
    """显示 base64 图像"""

    # 使用 base64 字符串创建 HTML img 标签
    image_html = f'<img src="data:image/jpeg;base64,{img_base64}" />'

    # 通过渲染 HTML 显示图像
    # display(HTML(image_html))

curr_dir = os.path.dirname(os.path.abspath(__file__))

path = os.path.join(curr_dir, "dualarm.png")
img_base64 = encode_image(path)
# plt_img_base64(img_base64)

from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI

# chat = ChatOpenAI(model="yi-vision",max_tokens=1024, base_url='https://api.lingyiwanwu.com/v1', api_key=zero_one_api)
chat = ChatOpenAI(model="gpt-4o")

msg = chat.invoke(
    [
        HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": "Now you are this robot. Can you prepare a cup of juice for me? list the actions of your arms separately in the order of execution.",
                },
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"},
                },
            ]
        )
    ]
)
from pprint import pprint
pprint(msg.content)