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
finetuning_messages = finetuning_messages[:10]

# Write finetuning_messages to a jsonl file
with open("/home/blackbird/kios/data/finetuning/finetuning_messages.jsonl", "w") as f:
    for msg in finetuning_messages:
        json.dump(msg, f)
        f.write("\n")

from openai import OpenAI
import os

openai_client = OpenAI()

training_file = openai_client.files.create(
    file=open(
        os.path.join(os.path.dirname(__file__), "finetuning_messages.jsonl"),
        "rb",
    ),
    purpose="fine-tune",
)

# # Wait while the file is processed
status = openai_client.files.retrieve("file-EhbEPGHSCMj1bD6TGi6WQMxT").status
# start_time = time.time()
# while status != "processed":
#     print(f"Status=[{status}]... {time.time() - start_time:.2f}s", end="\r", flush=True)
#     time.sleep(5)
#     status = openai.File.retrieve(training_file.id).status
# print(f"File {training_file.id} ready after {time.time() - start_time:.2f} seconds.")

# from openai import OpenAI

# client = OpenAI()

# client.fine_tuning.jobs.create(training_file="UT_gen_test", model="gpt-3.5-turbo")
