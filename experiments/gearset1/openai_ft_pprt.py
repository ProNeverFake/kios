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

cur_dir = os.path.dirname(__file__)
ground_truth_file = os.path.join(cur_dir, "baseline_result.jsonl")

with open(ground_truth_file, "r") as f:
    ground_truth_list = f.readlines()

baseline_results = [json.loads(result) for result in ground_truth_list]


def convert_openai_ft_msg(user: str, ai: str) -> dict[str, list[dict[str, str]]]:
    return {
        "messages": [
            {
                "role": "system",
                "content": "You are an excellent behavior tree builder for robotic assembly tasks",
            },
            {
                "role": "user",
                "content": user,
            },
            {
                "role": "assistant",
                "content": ai,
            },
        ]
    }


final_data = []

for i in range(len(baseline_results)):
    entry = baseline_results[i]
    initial_world_state = entry["initial_world_state"]
    target = entry["target"]
    ground_truth = entry["result"]

    prompt = e2e_ppt_ppl.invoke(
        {
            "target": target,
            "initial_state": initial_world_state,
        }
    )

    final_data.append(
        convert_openai_ft_msg(
            user=prompt.to_string(),
            ai=str(ground_truth),
        )
    )

with open(os.path.join(cur_dir, "openai_ft_one_step.jsonl"), "w") as f:
    for data in final_data:
        f.write(json.dumps(data) + "\n")
