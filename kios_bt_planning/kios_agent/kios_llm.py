from typing import Any, List, Dict  # ! use embedded typing in python from 3.10.
import openai
import tiktoken
import json
import os
import re
import ast
import argparse
import sys
import textwrap

VALID_API_VERSIONS = ["2022-12-01", "2023-05-15"]


class KiosLLM:
    encoder: tiktoken.Encoding = None
    prompt_directories: Dict[str, str] = None

    ######################################## *
    max_token_length: int
    max_completion_length: int
    last_response: str = None

    ######################################## * prompts
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
    messages: List[str] = None

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
    def initialize(self):
        self.encoder = tiktoken.get_encoding("cl100k_base")
        script_dir = os.path.dirname(__file__)
        prompt_dir = os.path.join(script_dir, "prompts")
        self.prompt_directories = {
            "system": os.path.join(prompt_dir, "system"),
            "query": os.path.join(prompt_dir, "query"),
            "prompt": os.path.join(prompt_dir, "prompt"),
        }
        # default prompt load order
        self.prompt_load_order = [
            "prompt_role",  # your are a good interpreter ... Done
            "prompt_domain",  # domain knowledge ... Done
            "prompt_problem",  # how the problem is provided ... Done
            "prompt_environment",  # how to express the environment ... Done
            "prompt_behaviortree",  # how to construct the behavior tree ... Done
            "prompt_output_format",  # the output format ... Done
            "prompt_example",  # some examples ... Done
        ]

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

    def query_llm(self, problem: str) -> dict:
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

        # * request the gpt to response
        response = openai.chat.completions.create(
            # model="gpt-3.5-turbo-16k-0613",
            # model="gpt-4-0613", # not this
            model="gpt-4-1106-preview",
            messages=self.create_prompt(),
            temperature=0.1,
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
        # dump to a text file
        with open("last_response.txt", "w") as f:
            f.write(self.last_response)
            try:
                self.json_dict = json.loads(self.last_response, strict=False)
                # update the environment after the execution of the action
                self.environment = self.json_dict["environment_after"]
            except json.JSONDecodeError as e:
                print("Error decoding JSON:", e)
                print("Problematic part:", self.last_response[e.pos - 10 : e.pos + 10])

        # * use the last response as the assistant prompt for the next round
        if len(self.messages) > 0 and self.last_response is not None:
            self.messages.append({"sender": "assistant", "text": self.last_response})

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


def test_llm():
    problem_name = "test_problem"
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

    if not os.path.exists("./out/" + problem_name):
        os.makedirs("./out/" + problem_name)

    # run the instructions one by one
    response = llm_model.query_llm(problem)

    llm_model.dump_json(f"./out/{problem_name}/0")


def extract_json_part(text):
    """
    extract the markdown code block part from the text
    """
    if text.find("```") == -1:
        return text
    text_json = text[text.find("```") + 3 : text.find("```", text.find("```") + 3)]
    return text_json


def test_json_file():
    current_dir = os.path.dirname(__file__)

    with open(os.path.join(current_dir, "the_string.txt"), "r") as f:
        text = f.read()
    print(text)
    last_response = text
    last_response = extract_json_part(last_response)
    last_response = last_response.replace("'", '"')
    # dump to a text file
    with open("last_response.txt", "w") as f:
        f.write(last_response)
        try:
            json_dict = ast.literal_eval(last_response)
            # json_dict = json.loads(last_response, strict=False)
            # update the environment after the execution of the action
            environment = json_dict["environment_after"]
        except json.JSONDecodeError as e:
            print("Error decoding JSON:", e)
            print("Problematic part:", last_response[e.pos - 10 : e.pos + 10])
            raise e

    # * use the last response as the assistant prompt for the next round
    # if len(self.messages) > 0 and self.last_response is not None:
    #     self.messages.append({"sender": "assistant", "text": self.last_response})

    # print(json_dict)
    print("the behavior tree is:")
    print(json_dict["task_cohesion"]["behavior_tree"])
    print("the environment after the action is:")
    print(environment)


if __name__ == "__main__":
    # test_llm()

    test_json_file()

    # # ! CHEAT
    # scenario_name = "gearset_1"
    # problem = None

    # if scenario_name == "chair_assembly_bt":
    #     problem = "(define (problem robot_assembly_problem-problem)\
    #                     (:domain robot_assembly_problem-domain)\
    #                     (:objects\
    #                     parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
    #                     gear1 gear2 gear3 shaft1 shaft2 base - part\
    #                     left_hand - hand\
    #                     )\
    #                     (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft2 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
    #                     (:goal (and (is_inserted_to gear3 shaft2)))\
    #                     )\
    #                     "
    # if scenario_name == "gearset_1":
    #     problem = "(define (problem robot_assembly_problem-problem)\
    #                     (:domain robot_assembly_problem-domain)\
    #                     (:objects\
    #                         parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
    #                         gear1 gear2 gear3 shaft1 shaft2 base - part\
    #                         left_hand - hand\
    #                     )\
    #                     (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft1 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
    #                     (:goal (and (is_inserted_to gear1 shaft1)))\
    #                     )"  # ! CHEAT

    #     # formatted_problem = textwrap.dedent(problem)
    #     # print(formatted_problem)
    # else:
    #     parser.error("Invalid scenario name:" + scenario_name)

    # aimodel = ChatGPT(
    #     credentials,
    #     prompt_load_order=prompt_load_order,
    #     use_azure=False,
    #     api_version="2023-05-15",
    # )

    # if not os.path.exists("./out/" + scenario_name):
    #     os.makedirs("./out/" + scenario_name)
    # # run the instructions one by one
    # text = aimodel.generate_with_problem(problem, is_user_feedback=False)
    # while True:
    #     user_feedback = input("user feedback (return empty if satisfied): ")
    #     if user_feedback == "q":
    #         exit()
    #     if user_feedback != "":
    #         text = aimodel.generate(user_feedback, environment, is_user_feedback=True)
    #     else:
    #         # update the current environment
    #         environment = aimodel.environment
    #         break
    # aimodel.dump_json(f"./out/{scenario_name}/0")
