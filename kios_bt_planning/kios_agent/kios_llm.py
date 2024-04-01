from typing import Any, List, Dict
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

"""
the llm class for prompt testing.
# ! deprecated
"""

# langsmith tracing
os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"


class KiosLLM:
    encoder: tiktoken.Encoding = None
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
    system_message: dict = None

    # The list of messages for prompting, will be sent to api
    # including: the required format (in prompt folder), the last response as assistant prompt
    messages: List[Dict[str, str]] = None

    ######################################## * dir
    history_dir: str = None

    def __init__(self, openai_api_key: str = None):
        if openai_api_key is not None:
            openai.api_key = openai_api_key
        else:
            try:
                openai.api_key = os.environ["OPENAI_API_KEY"]
            except KeyError:
                raise Exception(
                    "OPENAI_API_KEY is not given or set in the environment!"
                )

    # ! extend this method later.
    def initialize(self, prompt_dir: str = None, prompt_load_order: List[str] = None):
        self.encoder = tiktoken.get_encoding("cl100k_base")
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
            self.prompt_load_order = [
                "prompt_role",  # your are a good interpreter ... Done
                "prompt_domain",  # domain knowledge ... Done
                "prompt_problem",  # how the problem is provided ... Done
                "prompt_environment",  # how to express the environment ... Done
                # "prompt_behaviortree",  # how to construct the behavior tree ... Done
                "prompt_bt_skeleton",  # how to construct the skeleton behavior tree ... Done
                "prompt_bt_skeleton_example",  # some skeleton examples ... Done
                "prompt_output_format",  # the output format ... Done
                # "prompt_example",  # some examples ... Done
            ]

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
            data = f.read()
        self.system_message = {"role": "system", "content": data}

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
            for i, item in enumerate(data_spilit):
                if i % 2 == 0:
                    self.messages.append({"sender": "user", "text": item})
                else:
                    self.messages.append({"sender": "assistant", "text": item})

        # load the query template
        fp_query = os.path.join(self.prompt_directories["query"], "query.txt")
        with open(fp_query) as f:
            self.query = f.read()

        # print("KiosLLM initialized")
        # print(self.query)
        # print(self.messages)

    def setup_model(self):
        pass

    def model_query(self) -> json:
        pass

    def parse_response(self, response: json) -> Any:
        pass

    def create_prompt(self):
        """
        create the prompt from messages for gpt api.
        if too long, truncate the prompt and call this recursively
        """
        prompt = []  # list of prompt entries
        # load the system prompt
        prompt.append(self.system_message)
        # load the knowledge and the last response
        for message in self.messages:
            prompt.append({"role": message["sender"], "content": message["text"]})

        # * truncate the prompt if it is too long
        prompt_content = ""  # this is only for checking the length of the prompt
        for message in prompt:
            prompt_content += message["content"]
        print("prompt length: " + str(len(self.encoder.encode(prompt_content))))
        if (
            len(self.encoder.encode(prompt_content))
            > self.max_token_length - self.max_completion_length
        ):
            print("prompt too long. truncated.")
            # * truncate the prompt by removing the oldest two messages from front
            self.messages = self.messages[2:]
            prompt = self.create_prompt()
        return prompt

    def query_llm(self, problem: str, problem_name: str, model: str = None) -> dict:
        """
        runtime_instruction: the instruction from the problem/user
        """

        # get the query template
        text_base = self.query

        # replace problem part
        if text_base.find("[PROBLEM]") != -1:
            text_base = text_base.replace("[PROBLEM]", json.dumps(problem))

        # finally, add the text_base to the user query message
        self.messages.append({"sender": "user", "text": text_base})

        # print("messages:")
        # # print(self.messages)
        # for message in self.create_prompt():
        #     print(message.get("role"))

        # * Substitue the openai.Client() with wrap_openai(openai.Client()) to enable tracing
        client = wrap_openai(openai.Client())

        # * request the gpt to response
        if model is None:
            model = "gpt-4-1106-preview"

        response = client.chat.completions.create(
            # model="gpt-3.5-turbo-16k-0613",
            # model="gpt-4-0613", # not this
            model=model,
            messages=self.create_prompt(),
            temperature=0.0,
            max_tokens=self.max_completion_length,
            top_p=0.3,
            frequency_penalty=0.0,
            presence_penalty=0.3,
        )

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

        while not os.path.exists(os.path.join(self.history_dir, problem_name)):
            os.makedirs(os.path.join(self.history_dir, problem_name))

        problem_dir = os.path.join(self.history_dir, problem_name)

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


def test_llm():
    problem_name = "gearset_skeleton_v1"
    problem = "(define (problem robot_assembly_problem-problem)\
                    (:domain robot_assembly_problem-domain)\
                    (:objects\
                        parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
                        gear1 gear2 gear3 shaft1 shaft2 base - part\
                        left_hand - hand\
                    )\
                    (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft1 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
                    (:goal (and (is_inserted_to gear1 shaft1)))\
                    )"  # ! CHEAT

    llm_model = KiosLLM()
    llm_model.initialize()

    # run the instructions one by one
    response = llm_model.query_llm(problem, problem_name)


if __name__ == "__main__":
    # test_llm()
    pass
