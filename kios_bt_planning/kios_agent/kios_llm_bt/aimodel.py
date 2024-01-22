import openai
import tiktoken
import json
import os
import re
import argparse
import sys


enc = tiktoken.get_encoding("cl100k_base")

script_dir = os.path.dirname(__file__)
upper_level = os.path.dirname(os.path.dirname(script_dir))

secret_path = os.path.join(upper_level, "secrets.json")

# with open("../../secrets.json") as f:
with open(secret_path) as f:
    credentials = json.load(f)
    # print(credentials)


# dir_system = "./system"
dir_system = os.path.join(script_dir, "system")
# dir_prompt = "./prompt"
dir_prompt = os.path.join(script_dir, "prompt")
# dir_query = "./query"
dir_query = os.path.join(script_dir, "query")

prompt_load_order = [
    "prompt_role",  # your are a good interpreter ... Done
    "prompt_domain",  # domain knowledge ... Done
    "prompt_problem",  # how the problem is provided ... Done
    "prompt_environment",  # how to express the environment ... Done
    "prompt_behaviortree",  # how to construct the behavior tree ... Done
    "prompt_output_format",  # the output format ... Done
    "prompt_example",  # some examples ... Done
]

# and also, query: the user request and the rules for the generation of the response ... Done
# ! still not rules about the behavior tree yet.


# azure openai api has changed from '2023-05-15' to '2023-05-15'
# if you are using a 0301 version, use '2022-12-01'
# Otherwise, use '2023-05-15'


class ChatGPT:
    problem: str = None
    """
    the problem description 
    """

    query: str = None
    """
        the query request: start working ...
    """
    instruction: str = None
    """
    the member variable to keep the latest instruction
    """
    system_message: dict = None
    """
    the "system" prompt part
    """
    messages: list = None
    """
    The list of messages for prompting.
    including: the required format (in prompt folder), the last response as assistant prompt
    """

    VALID_API_VERSIONS = ["2022-12-01", "2023-05-15"]

    def __init__(
        self, credentials, prompt_load_order, use_azure=False, api_version="2023-05-15"
    ):
        # initialize the openai api
        self.use_azure = use_azure
        if self.use_azure:
            openai.api_key = credentials["azureopenai"]["AZURE_OPENAI_KEY"]
            openai.api_base = credentials["azureopenai"]["AZURE_OPENAI_ENDPOINT"]
            openai.api_type = "azure"
            if api_version not in self.VALID_API_VERSIONS:
                raise ValueError(
                    f"api_version must be one of {self.VALID_API_VERSIONS}"
                )
            openai.api_version = api_version
        else:
            openai.organization = credentials["openai"]["YOUR_ORG_ID"]
            openai.api_key = credentials["openai"]["OPENAI_API_KEY"]

        self.credentials = credentials
        self.messages = []
        self.max_token_length = 25000
        self.max_completion_length = 4000
        self.last_response = None
        self.query = ""
        self.instruction = ""
        # load prompt file
        # * system
        fp_system = os.path.join(dir_system, "system.txt")
        with open(fp_system) as f:
            data = f.read()
        self.system_message = {"role": "system", "content": data}

        # load prompt file
        for prompt_name in prompt_load_order:
            fp_prompt = os.path.join(dir_prompt, prompt_name + ".txt")
            with open(fp_prompt) as f:
                data = f.read()
            data_spilit = re.split(r"\[user\]\n|\[assistant\]\n", data)
            data_spilit = [item for item in data_spilit if len(item) != 0]
            # it start with user and ends with system
            assert len(data_spilit) % 2 == 0
            for i, item in enumerate(data_spilit):
                if i % 2 == 0:
                    self.messages.append({"sender": "user", "text": item})
                else:
                    self.messages.append({"sender": "assistant", "text": item})
        fp_query = os.path.join(dir_query, "query.txt")
        with open(fp_query) as f:
            self.query = f.read()

    def create_prompt(self):
        """
        create the prompt for gpt api
        """
        prompt = ""
        if self.use_azure and openai.api_version == "2022-12-01":
            prompt = "<|im_start|>system\n"
            prompt += self.system_message["content"]
            prompt += "\n<|im_end|>\n"
            for message in self.messages:
                prompt += (
                    f"\n<|im_start|>{message['sender']}\n{message['text']}\n<|im_end|>"
                )
            prompt += "\n<|im_start|>assistant\n"
            print("prompt length: " + str(len(enc.encode(prompt))))
            if (
                len(enc.encode(prompt))
                > self.max_token_length - self.max_completion_length
            ):
                print("prompt too long. truncated.")
                # * truncate the prompt by removing the oldest two messages
                self.messages = self.messages[2:]
                prompt = self.create_prompt()
        else:
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
            print("prompt length: " + str(len(enc.encode(prompt_content))))
            if (
                len(enc.encode(prompt_content))
                > self.max_token_length - self.max_completion_length
            ):
                print("prompt too long. truncated.")
                # * truncate the prompt by removing the oldest two messages
                self.messages = self.messages[2:]
                prompt = self.create_prompt()
        return prompt

    def extract_json_part(self, text):
        """
        extract the markdown code block part from the text
        """
        if text.find("```") == -1:
            return text
        text_json = text[text.find("```") + 3 : text.find("```", text.find("```") + 3)]
        return text_json

    def generate(self, runtime_instruction, environment, is_user_feedback=False):
        """
        runtime_instruction: the instruction from the problem/user
        """
        if is_user_feedback:
            self.messages.append(
                {
                    "sender": "user",
                    "text": runtime_instruction + "\n" + self.instruction,
                }
            )
        else:
            # get the query template
            text_base = self.query

            # # * bb
            # if text_base.find("[PROBLEM]") != -1:
            #     text_base = text_base.replace("[PROBLEM]", json.dumps(environment))

            # replace the environment part of the template
            if text_base.find("[ENVIRONMENT]") != -1:
                text_base = text_base.replace("[ENVIRONMENT]", json.dumps(environment))
            # replace the instruction part of the template
            if text_base.find("[INSTRUCTION]") != -1:
                text_base = text_base.replace("[INSTRUCTION]", runtime_instruction)
                # update the instruction of the chatbot
                self.instruction = text_base
            # finally, add the text_base to the user query message
            self.messages.append({"sender": "user", "text": text_base})

        if self.use_azure and openai.api_version == "2022-12-01":
            # Remove unsafe user inputs. May need further refinement in the
            # future.
            if runtime_instruction.find("<|im_start|>") != -1:
                runtime_instruction = runtime_instruction.replace("<|im_start|>", "")
            if runtime_instruction.find("<|im_end|>") != -1:
                runtime_instruction = runtime_instruction.replace("<|im_end|>", "")
            deployment_name = self.credentials["azureopenai"][
                "AZURE_OPENAI_DEPLOYMENT_NAME_CHATGPT"
            ]
            response = openai.Completion.create(
                engine=deployment_name,
                prompt=self.create_prompt(),
                temperature=0.1,
                max_tokens=self.max_completion_length,
                top_p=0.5,
                frequency_penalty=0.0,
                presence_penalty=0.0,
                stop=["<|im_end|>"],
            )
            text = response["choices"][0]["text"]
        elif self.use_azure and openai.api_version == "2023-05-15":
            deployment_name = self.credentials["azureopenai"][
                "AZURE_OPENAI_DEPLOYMENT_NAME_CHATGPT"
            ]
            response = openai.ChatCompletion.create(
                engine=deployment_name,
                messages=self.create_prompt(),
                temperature=0.1,
                max_tokens=self.max_completion_length,
                top_p=0.5,
                frequency_penalty=0.0,
                presence_penalty=0.0,
            )
            text = response["choices"][0]["message"]["content"]
        else:
            # * request the gpt to response
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo-16k",
                # "gpt-4" is available, too. Check the available models in https://platform.openai.com/docs/models/
                messages=self.create_prompt(),
                temperature=0.1,
                max_tokens=self.max_completion_length,
                top_p=0.5,
                frequency_penalty=0.0,
                presence_penalty=0.0,
            )
            text = response["choices"][0].message.content
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
        except BaseException:
            self.json_dict = None
            import pdb

            pdb.set_trace()

        # * use the last response as the assistant prompt for the next round
        if len(self.messages) > 0 and self.last_response is not None:
            self.messages.append({"sender": "assistant", "text": self.last_response})

        return self.json_dict

    def generate_with_problem(self, problem, is_user_feedback=False):
        """
        runtime_instruction: the instruction from the problem/user
        """
        if is_user_feedback:
            pass
        else:
            # get the query template
            text_base = self.query

            # * bb
            if text_base.find("[PROBLEM]") != -1:
                text_base = text_base.replace("[PROBLEM]", json.dumps(problem))

            # finally, add the text_base to the user query message
            self.messages.append({"sender": "user", "text": text_base})

        # * request the gpt to response
        response = openai.chat.completions.create(
            # model="gpt-3.5-turbo-16k-0613",
            # model="gpt-4-0613", # not this
            model="gpt-4-1106-preview",
            # "gpt-4" is available, too. Check the available models in https://platform.openai.com/docs/models/
            messages=self.create_prompt(),
            temperature=0.1,
            max_tokens=self.max_completion_length,
            top_p=0.5,
            frequency_penalty=0.0,
            presence_penalty=0.0,
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

        # self.json_dict = None
        # import pdb

        # pdb.set_trace()

        # * use the last response as the assistant prompt for the next round
        if len(self.messages) > 0 and self.last_response is not None:
            self.messages.append({"sender": "assistant", "text": self.last_response})

        return self.json_dict

    def dump_json(self, dump_name=None):
        """
        dump the json dictionary to a file, used to save into a json file
        """
        if dump_name is not None:
            # dump the dictionary to json file dump 1, 2, ...
            fp = os.path.join(dump_name + ".json")
            with open(fp, "w") as f:
                json.dump(self.json_dict, f, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        type=str,
        required=False,  # ! CHEAT
        help="scenario name (see the code for details)",
    )
    args = parser.parse_args()
    scenario_name = args.scenario

    # ! CHEAT
    scenario_name = "chair_assembly_bt"
    problem = None

    # 1. example of moving objects on the table and the shelf
    if scenario_name == "shelf":
        environment = {
            "assets": [
                "<table>",
                "<shelf_bottom>",
                "<shelf_top>",
                "<trash_bin>",
                "<floor>",
            ],
            "asset_states": {
                "<shelf_bottom>": "on_something(<table>)",
                "<trash_bin>": "on_something(<floor>)",
            },
            "objects": ["<spam>", "<juice>"],
            "object_states": {
                "<spam>": "on_something(<table>)",
                "<juice>": "on_something(<shelf_bottom>)",
            },
        }
        instructions = [
            "Put the juice on top of the shelf",
            "Throw away the spam into the trash bin",
            "Move the juice on top of the table",
            "Throw away the juice",
        ]
    # 2. example of opening and closing the fridge, and putting the juice on
    # the floor
    elif scenario_name == "fridge":
        environment = {
            "assets": ["<fridge>", "<floor>"],
            "asset_states": {"<fridge>": "on_something(<floor>)"},
            "objects": ["<fridge_handle>", "<juice>"],
            "object_states": {
                "<fridge_handle>": "closed()",
                "<juice>": "inside_something(<fridge>)",
            },
        }
        instructions = [
            "Open the fridge half way",
            "Open the fridge wider",
            "Take the juice in the fridge and put it on the floor",
            "Close the fridge",
        ]
    # 3. example of opening and closing the drawer
    elif scenario_name == "drawer":
        environment = {
            "assets": ["<drawer>", "<floor>"],
            "asset_states": {"<drawer>": "on_something(<floor>)"},
            "objects": ["<drawer_handle>"],
            "object_states": {"<drawer_handle>": "closed()"},
        }
        instructions = [
            "Open the drawer widely",
            "Close the drawer half way",
            "Close the drawer fully",
        ]
    # 4. example of wiping the table
    elif scenario_name == "table":
        environment = {
            "assets": ["<table1>", "<table2>", "<trash_bin>", "<floor>"],
            "asset_states": {
                "<table1>": "next_to(<table2>)",
                "<trash_bin>": "on_something(<floor>)",
            },
            "objects": ["<sponge>"],
            "object_states": {"<sponge>": "on_something(<table1>)"},
        }
        instructions = [
            "Put the sponge on the table2",
            "Wipe the table2 with the sponge",
        ]
    # 5. example of wiping the window
    elif scenario_name == "window":
        environment = {
            "assets": ["<table>", "<window>", "<trash_bin>", "<floor>"],
            "asset_states": {
                "<table>": "next_to(<window>)",
                "<trash_bin>": "on_something(<floor>)",
            },
            "objects": ["<sponge>"],
            "object_states": {"<sponge>": "on_something(<table>)"},
        }
        instructions = [
            "Get the sponge from the table and wipe the window with it. After that, put the sponge back on the table",
            "Throw away the sponge on the table",
        ]
    elif scenario_name == "chair_assembly_bt":
        problem = "(define (problem robot_assembly_problem-problem)\
                        (:domain robot_assembly_problem-domain)\
                        (:objects\
                            parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool\
                            gear1 gear2 gear3 shaft1 shaft2 base - part\
                            left_hand - hand\
                        )\
                        (:init (can_manipulate parallel_box1 gear1) (can_manipulate outward_claw gear2) (can_manipulate inward_claw gear3) (can_manipulate parallel_box2 shaft1) (can_manipulate no_tool shaft2) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub base) (can_place_to lamp blub) (can_insert_to shaft1 base) (can_insert_to shaft2 base) (can_insert_to gear3 shaft2) (can_insert_to gear2 base) (can_insert_to gear1 shaft1) (is_inserted_to shaft2 base) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_free gear1) (is_free gear2) (is_free gear3) (is_free shaft1) (is_free shaft2) (is_free base) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))\
                        (:goal (and (is_inserted_to gear3 shaft2)))\
                    )"

    else:
        parser.error("Invalid scenario name:" + scenario_name)

    aimodel = ChatGPT(
        credentials,
        prompt_load_order=prompt_load_order,
        use_azure=False,
        api_version="2023-05-15",
    )

    if not os.path.exists("./out/" + scenario_name):
        os.makedirs("./out/" + scenario_name)
    # run the instructions one by one
    text = aimodel.generate_with_problem(problem, is_user_feedback=False)
    while True:
        user_feedback = input("user feedback (return empty if satisfied): ")
        if user_feedback == "q":
            exit()
        if user_feedback != "":
            text = aimodel.generate(user_feedback, environment, is_user_feedback=True)
        else:
            # update the current environment
            environment = aimodel.environment
            break
    aimodel.dump_json(f"./out/{scenario_name}/0")
