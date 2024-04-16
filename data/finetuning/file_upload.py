from openai import OpenAI

client = OpenAI()

import os

cur_dir = os.path.dirname(__file__)
file_dir = os.path.join(cur_dir, "openai_ft_one_step.jsonl")

client.files.create(file=open(file_dir, "rb"), purpose="fine-tune")
