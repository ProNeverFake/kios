import os
import json
import dotenv

dotenv.load_dotenv()

from langchain.prompts.prompt import PromptTemplate
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import JsonOutputParser, StrOutputParser
from langchain.prompts.pipeline import PipelinePromptTemplate


data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
prompt_dir = os.path.join(data_dir, "prompts")

system_file = os.path.join(prompt_dir, "end_to_end_v3/system.txt")
task_file = os.path.join(prompt_dir, "end_to_end_v3/task.txt")
domain_file = os.path.join(prompt_dir, "end_to_end_v3/domain.txt")
behaviortree_file = os.path.join(prompt_dir, "end_to_end_v3/behaviortree.txt")
# example_file = os.path.join(prompt_dir, "end_to_end_v3/example.txt")
output_format_file = os.path.join(prompt_dir, "end_to_end_v3/output_format.txt")
state_file = os.path.join(prompt_dir, "end_to_end_v3/state.txt")
template_file = os.path.join(prompt_dir, "end_to_end_v3/template.txt")
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
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())
# with open(example_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     example_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {behaviortree}

    {output_format}

    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

e2e_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("behaviortree", behaviortree_ppt),
        ("output_format", output_format_ppt),
        ("state", state_ppt),
        ("format_instructions", format_instructions),
    ],
)

# from pprint import pprint

# response = e2e_ppt_ppl.invoke(
#     {
#         "target": "THIS IS THE TARGET",
#         "initial_state": "THIS IS THE INITIAL STATE",
#     }
# )

# rs = response.to_string()


current_dir = os.path.dirname(os.path.abspath(__file__))

baseline_file_dir = os.path.join(current_dir, "baseline_result.jsonl")

with open(baseline_file_dir, "r") as f:
    baseline_results = f.readlines()

baseline_results = [json.loads(result) for result in baseline_results]

# result

# initial_world_state

# target

final_data = []

for i in range(len(baseline_results)):
    entry = baseline_results[i]
    initial_world_state = entry["initial_world_state"]
    target = entry["target"]
    ground_truth = entry["result"]

    response = e2e_ppt_ppl.invoke(
        {
            "target": target,
            "initial_state": initial_world_state,
        }
    )

    final_data.append(
        {
            "system": "You are an excellent interpreter of instructions for robotic assembly tasks.",
            "instruction": response.to_string(),
            "output": json.dumps(ground_truth),
            "input": "",
        }
    )

with open(os.path.join(current_dir, "ft_data_one_step.jsonl"), "w") as f:
    for entry in final_data:
        f.write(json.dumps(entry) + "\n")
