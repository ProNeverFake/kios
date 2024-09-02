import json
import os

"""

"""
# http proxy of the clash
# os.environ["http_proxy"] = "http://127.0.0.1:7890"
# os.environ["https_proxy"] = "http://127.0.0.1:7890"

# MEGVII http_proxy transparent proxy
# os.environ["http_proxy"] = "http://127.0.0.1:80"
# os.environ["https_proxy"] = "https://127.0.0.1:443"

# os.environ["LANGCHAIN_TRACING_V2"] = "false"
# os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
# os.environ["LANGCHAIN_PROJECT"] = "kios_dualarm"


from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from dotenv import load_dotenv

from langchain_openai import ChatOpenAI

from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import JsonOutputParser

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate

from langsmith import traceable

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree


def render_bt(bt_json: dict):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)


####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
world_state_path = os.path.join(current_dir, "world_state.json")

####################### world
world_interface = WorldInterface()
with open(world_state_path, "r") as file:
    world_state_json = json.load(file)

####################### robot
robot_interface = RobotInterface(
    robot_address="127.0.0.1",
    robot_port=12000,
)

####################### bt_factory
bt_factory = BehaviorTreeFactory(
    world_interface=world_interface,
    robot_interface=robot_interface,
)

####################### behavior_tree_stewardship
behavior_tree_stewardship = BehaviorTreeStewardship(
    behaviortree_factory=bt_factory,
    world_interface=world_interface,
    robot_interface=robot_interface,
)

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
print(data_dir)
prompt_dir = os.path.join(data_dir, "prompts")
folder_name = "dualarm3"
prompt_dir = os.path.join(prompt_dir, folder_name)

# * dualarm bt generation ppl
system_file = os.path.join(prompt_dir, "system.txt")
task_file = os.path.join(prompt_dir, "task.txt")
domain_file = os.path.join(prompt_dir, "domain.txt")
behaviortree_file = os.path.join(prompt_dir, "behaviortree.txt")
# example_file = os.path.join(prompt_dir, "dualarm/example.txt")
# output_format_file = os.path.join(prompt_dir, "dualarm/output_format.txt")
state_file = os.path.join(prompt_dir, "state.txt")
template_file = os.path.join(prompt_dir, "template.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(state_file, "r") as f:
    state_ppt = PromptTemplate.from_template(f.read())
# with open(output_format_file, "r") as f:
# output_format_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())
# with open(example_file, "r") as f:
# ppt_tmp = PromptTemplate.from_template("{input}")
# example_ppt = ppt_tmp.partial(input=f.read())

from langchain_core.prompts.image import ImagePromptTemplate

image_ppt = ImagePromptTemplate(input_variables = ["image_url"],
                                # input_types = {"image": "image_url"},
                                )

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {image}

    {behaviortree}

    {template}

    {format_instructions}
    """
    # remove state here
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

dualarm_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("behaviortree", behaviortree_ppt),
        # ("output_format", output_format_ppt),
        # ("example", example_ppt),
        # ("state", state_ppt),
        ("format_instructions", format_instructions),
        ("image", image_ppt),
    ],
)


zero_one_api = os.environ.get("ZERO_ONE_API")

dualarm_chain = (
    dualarm_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="gpt-4o", temperature=0)
    | ChatOpenAI(model="yi-vision", max_tokens=1024, base_url="https://api.lingyiwanwu.com/v1", api_key=zero_one_api)
    | JsonOutputParser()
)

instruction = """
The left arm reaches the glass on the table1, grasps it and retreats back.
The right arm reaches the water bottle on the table2, grasps it and retreats back.
The right arm then pours the water in the water bottle into the glass. 
If the glass is not ready, the right arm waits for it.
After this, the left arm reaches the table1 and release the glass. The right arm reaches the table2 and release the water bottle.
"""

import base64

def encode_image(image_path):
    """获取图像的 base64 字符串"""

    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")
    

path = os.path.join(current_dir, "dualarm.png")
img_base64 = encode_image(path)

def rollout():

    # record the voice and translate it into chinese
    # from audio import record_and_transcribe

    # instruction = record_and_transcribe(lang="en")

    # invoke to generate the behavior tree
    bt = dualarm_chain.invoke(
        {
            "instruction": instruction,
            # "initial_state": world_state_json,
            "initial_state": {},
            # "image_url": path,
            # "image_url": img_base64,
            "image_url": "file://" + path,
        }
    )

    print(bt)
    render_bt(bt)

    # send the behavior tree to localhost:8004
    from zmq_manager import ZMQManager

    manager = ZMQManager(host="127.0.0.1", port=8004)
    manager.send_json(bt)


if __name__ == "__main__":
    rollout()

"""
HERE THE INSTRUCTION FOR INPUT:

The left arm reaches the glass on the table1, grasps it and retreats back.
The right arm reaches the water bottle on the table2, grasps it and retreats back.
The right arm then pours the water in the water bottle into the glass. 
If the glass is not ready, the right arm waits for it.
After this, the left arm reaches the table1 and release the glass. The right arm reaches the table2 and release the water bottle.
"""

from kios_utils.parsers import match_type

def tree_to_queue(tree_node: dict, sequence_list: list) -> dict:
    '''
    tree: the behavior tree in json format
    queue_dict:
        {
            left_queue: [],
            right_queue: [],
            normal_queue: []
        }
    
    return: queue_dict
    '''
    node_type, node_body = match_type(tree_node)

    if node_type == "sequence":
        queue_dict["normal_queue"].append(tree_node)
    



example_bt = {
    "summary": "sequence to perform the task of manipulating the glass and water bottle with left and right arms",
    "name": "sequence: main_task",
    "children": [
        {
            "summary": "parallel to perform initial reaching and grasping tasks",
            "name": "parallel: initial_reach_and_grasp",
            "children": [
                {
                    "summary": "sequence for left arm to reach and grasp the glass on table1",
                    "name": "sequence: left_arm_reach_and_grasp_glass",
                    "children": [
                        {
                            "summary": "left arm reaches the glass on table1",
                            "name": "action: REACH(left_arm, table1)",
                        },
                        {
                            "summary": "left arm grasps the glass",
                            "name": "action: GRASP(left_arm, glass)",
                        },
                        {
                            "summary": "left arm retreats with the glass",
                            "name": "action: RETREAT(left_arm)",
                        },
                    ],
                },
                {
                    "summary": "sequence for right arm to reach and grasp the water bottle on table2",
                    "name": "sequence: right_arm_reach_and_grasp_bottle",
                    "children": [
                        {
                            "summary": "right arm reaches the water bottle on table2",
                            "name": "action: REACH(right_arm, table2)",
                        },
                        {
                            "summary": "right arm grasps the water bottle",
                            "name": "action: GRASP(right_arm, watter_bottle)",
                        },
                        {
                            "summary": "right arm retreats with the water bottle",
                            "name": "action: RETREAT(right_arm)",
                        },
                    ],
                },
            ],
        },
        {
            "summary": "sequence for right arm to pour water into the glass",
            "name": "sequence: right_arm_pour_water",
            "children": [
                {
                    "summary": "right arm waits if the glass is not ready",
                    "name": "selector: wait_or_pour",
                    "children": [
                        {
                            "summary": "condition to check if the glass is ready",
                            "name": "condition: is_ready(glass)",
                        },
                        {
                            "summary": "right arm waits",
                            "name": "action: WAIT(right_arm)",
                        },
                    ],
                },
                {
                    "summary": "right arm pours water from the bottle into the glass",
                    "name": "action: POUR(right_arm, watter_bottle, glass)",
                },
            ],
        },
        {
            "summary": "parallel to perform final releasing tasks",
            "name": "parallel: final_release",
            "children": [
                {
                    "summary": "sequence for left arm to release the glass on table1",
                    "name": "sequence: left_arm_release_glass",
                    "children": [
                        {
                            "summary": "left arm reaches table1",
                            "name": "action: REACH(left_arm, table1)",
                        },
                        {
                            "summary": "left arm releases the glass",
                            "name": "action: RELEASE(left_arm, glass)",
                        },
                    ],
                },
                {
                    "summary": "sequence for right arm to release the water bottle on table2",
                    "name": "sequence: right_arm_release_bottle",
                    "children": [
                        {
                            "summary": "right arm reaches table2",
                            "name": "action: REACH(right_arm, table2)",
                        },
                        {
                            "summary": "right arm releases the water bottle",
                            "name": "action: RELEASE(right_arm, watter_bottle)",
                        },
                    ],
                },
            ],
        },
    ],
}
