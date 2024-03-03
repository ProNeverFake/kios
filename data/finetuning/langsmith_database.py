from langsmith import Client
from pprint import pprint
import json

client = Client()

# import datetime

# project_name = "kios_agent"
# run_type = "llm"
# end_time = datetime.datetime.now()

# runs = client.list_runs(
#     project_name=project_name,
#     run_type=run_type,
#     error=False,
# )

from langsmith import schemas
from langchain import load


def convert_messages(example: schemas.Example) -> dict:
    pprint(example.inputs)
    pprint(example.outputs)
    user_input = load.load(example.inputs)["input"][0]["data"]["content"]
    ai_output = load.load(example.outputs)["output"]["data"]["content"]
    return {
        "user": user_input,
        "ai": ai_output,
    }


def convert_openai_ft_msg(msg: dict) -> dict[str, list[dict[str, str]]]:
    return {
        "messages": [
            {
                "role": "system",
                "content": "You are an excellent behavior tree builder for robotic assembly tasks",
            },
            {
                "role": "user",
                "content": msg["user"],
            },
            {
                "role": "assistant",
                "content": msg["ai"],
            },
        ]
    }


messages = [
    convert_messages(example)
    for example in client.list_examples(dataset_name="UT-gen-data")
]

# from langchain.adapters import openai as openai_adapter

finetuning_messages = [convert_openai_ft_msg(msg) for msg in messages]

# pprint(finetuning_messages[0])

# ! just try
# finetuning_messages = finetuning_messages[:10]

# Write finetuning_messages to a jsonl file
with open("/home/blackbird/kios/data/finetuning/finetuning_messages_20240303.jsonl", "w") as f:
    for msg in finetuning_messages:
        json.dump(msg, f)
        f.write("\n")
