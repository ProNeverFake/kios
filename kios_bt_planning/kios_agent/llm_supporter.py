from typing import Any, List, Dict  # ! use embedded typing in python from 3.10.
from deprecated import deprecated

import openai
import tiktoken
import json
import os
import re
import ast
import argparse
import sys
import textwrap

from langsmith import traceable
from langsmith.wrappers import wrap_openai

from langchain_core.prompts import (
    ChatPromptTemplate,
    FewShotChatMessagePromptTemplate,
    BaseChatPromptTemplate,
)
from langchain_core.output_parsers import JsonOutputParser
from langchain_openai import ChatOpenAI


"""
the langchain version of the kios llm
"""

# langsmith tracing
os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"


class KiosLLMSupporter:
    prompt_directories: Dict[str, str] = None

    ######################################## *
    max_token_length: int
    max_completion_length: int
    last_response: str = None

    ######################################## * prompts
    prompt_dir: str = None
    prompt_load_order: List[str] = None

    # the problem description
    problem: str = None

    # the query template
    query: str = None

    # variable to keep the latest instruction
    instruction: str = None

    # the "system" prompt part
    system_message: str = None

    # The list of messages for prompting, will be sent to api
    # including: the required format (in prompt folder), the last response as assistant prompt
    messages: List[Dict[str, str]] = None

    ######################################## * dir
    history_dir: str = None

    def __init__(self):
        pass

    # ! extend this method later.
    def initialize(self, prompt_dir: str = None, prompt_load_order: List[str] = None):
        script_dir = os.path.dirname(__file__)
        if prompt_dir is not None:
            self.prompt_dir = os.path.join(script_dir, prompt_dir)
        else:
            self.prompt_dir = os.path.join(script_dir, "prompts")
        self.prompt_directories = {
            "system": os.path.join(self.prompt_dir, "system"),
            "query": os.path.join(self.prompt_dir, "query"),
            "prompt": os.path.join(self.prompt_dir, "prompt"),
        }

        # * check the prompt directories
        print("prompt directories:")
        for key, value in self.prompt_directories.items():
            print(f'["{key}"] = {value}')

        # default prompt load order
        if prompt_load_order is not None:
            self.prompt_load_order = prompt_load_order
        else:
            # self.prompt_load_order = [
            #     "prompt_role",  # your are a good interpreter ... Done
            #     "prompt_domain",  # domain knowledge ... Done
            #     "prompt_problem",  # how the problem is provided ... Done
            #     "prompt_environment",  # how to express the environment ... Done
            #     # "prompt_behaviortree",  # how to construct the behavior tree ... Done
            #     "prompt_bt_skeleton",  # how to construct the skeleton behavior tree ... Done
            #     "prompt_bt_skeleton_example",  # some skeleton examples ... Done
            #     "prompt_output_format",  # the output format ... Done
            #     # "prompt_example",  # some examples ... Done
            # ]
            raise Exception("prompt_load_order is not given!")

        # history directory
        if not os.path.exists(os.path.join(script_dir, "query_history")):
            os.makedirs(os.path.join(script_dir, "query_history"))
        self.history_dir = os.path.join(script_dir, "query_history")

        # default
        self.max_token_length: int = 16000
        self.max_completion_length: int = 4000
        self.last_response = None

        # prompt initialization
        self.messages = []
        self.query = ""
        self.instruction = ""

        # *load prompt file
        # system
        fp_system = os.path.join(self.prompt_directories["system"], "system.txt")
        with open(fp_system) as f:
            self.system_message = f.read()

        # prompt for domain knowledge, user: brabrabra, assistant: do nothing and wait for the next prompt
        for prompt_name in self.prompt_load_order:
            fp_prompt = os.path.join(
                self.prompt_directories["prompt"], prompt_name + ".txt"
            )
            with open(fp_prompt) as f:
                data = f.read()
            data_spilit = re.split(r"\[user\]\n|\[assistant\]\n", data)
            data_spilit = [item for item in data_spilit if len(item) != 0]
            assert len(data_spilit) % 2 == 0
            # load sparately to user and assistant
            message = {}
            for i, item in enumerate(data_spilit):
                if i % 2 == 0:
                    message["input"] = item
                else:
                    message["output"] = item
                    self.messages.append(message)
                    message = {}

        # load the query template
        fp_query = os.path.join(self.prompt_directories["query"], "query.txt")
        with open(fp_query) as f:
            self.query = f.read()

    def create_prompt(self):
        """
        create the prompt from messages for gpt api.
        if too long, truncate the prompt and call this recursively
        """

        example_template = ChatPromptTemplate.from_messages(
            [
                ("human", "{input}"),
                ("ai", "{output}"),
            ]
        )

        few_shot_prompt = FewShotChatMessagePromptTemplate(
            example_prompt=example_template,
            examples=self.messages,
        )

        system_prompt = ChatPromptTemplate.from_messages(
            [
                ("system", self.system_message),
            ]
        )

        query_prompt = ChatPromptTemplate.from_messages(
            [
                ("human", self.query),
            ]
        )

        final_prompt = ChatPromptTemplate.from_messages(
            [
                system_prompt,
                few_shot_prompt,
                query_prompt,
            ]
        )

        return final_prompt

    @deprecated(reason="use chain now")
    def query_llm(self, problem: str, problem_name: str, model: str = None) -> dict:

        if model is None:
            model = "gpt-4-1106-preview"

        # * Substitue the openai.Client() with wrap_openai(openai.Client()) to enable tracing
        llm = ChatOpenAI(
            model_name=model,
            temperature=0.0,
            max_tokens=self.max_completion_length,
        )

        # * request the gpt to response

        text = response.choices[0].message.content
        print(text)

        self.last_response = text
        self.last_response = self.extract_json_part(self.last_response)
        self.last_response = self.last_response.replace("'", '"')

        self.record_history(
            query=text_base, response=self.last_response, problem_name=problem_name
        )

        self.json_dict = json.loads(self.last_response, strict=False)

        return self.json_dict

    def extract_json_part(self, text):
        """
        extract the markdown code block part from the text
        """
        if text.find("```") == -1:
            return text
        text_json = text[text.find("```") + 3 : text.find("```", text.find("```") + 3)]
        return text_json

    def dump_json(self, dump_name=None):
        """
        dump the json dictionary to a file, used to save into a json file
        """
        if dump_name is not None:
            # dump the dictionary to json file dump 1, 2, ...
            fp = os.path.join(dump_name + ".json")
            with open(fp, "w") as f:
                json.dump(self.json_dict, f, indent=4)

    def record_history(self, query: str, response: str, problem_name: str):

        # * setup the problem directory
        while not os.path.exists(os.path.join(self.history_dir, problem_name)):
            os.makedirs(os.path.join(self.history_dir, problem_name))

        problem_dir = os.path.join(self.history_dir, problem_name)

        # * the try number under it
        i = 0
        while os.path.exists(os.path.join(problem_dir, str(i))):
            i = i + 1
        os.makedirs(os.path.join(problem_dir, str(i)))
        this_problem_dir = os.path.join(problem_dir, str(i))

        # get the json response
        json_response = response

        # dump to a text file
        with open(os.path.join(this_problem_dir, "response.txt"), "w") as f:
            f.write(json_response)

        # write the query problem
        with open(os.path.join(this_problem_dir, "query.txt"), "w") as f:
            f.write(query)

        # copy the prompts
        with open(os.path.join(this_problem_dir, "message.txt"), "w") as f:
            f.write(str(self.messages))


if __name__ == "__main__":
    # test_llm()
    pass
