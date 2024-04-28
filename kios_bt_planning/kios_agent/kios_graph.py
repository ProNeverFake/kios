import json
import os

# this should be set in the experiment file
# os.environ["LANGCHAIN_TRACING_V2"] = "true"
# os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
# os.environ["LANGCHAIN_PROJECT"] = "kios_human_in_the_loop"

"""
some chains that are used in kios graph
"""

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton

from kios_utils.bblab_utils import bb_deprecated

from dotenv import load_dotenv


from langchain_openai import ChatOpenAI

from langchain.chains.openai_functions import (
    create_structured_output_runnable,
)

from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import JsonOutputParser, StrOutputParser
from langchain_core.pydantic_v1 import BaseModel, Field

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate


load_dotenv()

####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "scene.json")
# bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_path = os.path.join(current_dir, "world_state.json")
problem_path = os.path.join(current_dir, "gearset.problem")
domain_knowledge_path = os.path.join(current_dir, "domain_knowledge.txt")

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
prompt_sk_dir = os.path.join(data_dir, "prompt_skeletons")
prompt_dir = os.path.join(data_dir, "prompts")

########## *sequential planner for BT

template_file = os.path.join(prompt_dir, "seq_planner/template.txt")
task_file = os.path.join(prompt_dir, "seq_planner/task.txt")
system_file = os.path.join(prompt_dir, "seq_planner/system.txt")
domain_file = os.path.join(prompt_dir, "seq_planner/domain.txt")
state_file = os.path.join(prompt_dir, "seq_planner/state.txt")
example_file = os.path.join(prompt_dir, "seq_planner/example.txt")
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
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}
    
    {example}

    {template}
    """
)

seq_planner_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("state", state_ppt),
        ("example", example_ppt),
        ("domain", domain_ppt),
    ],
)

seq_planner_chain = (
    seq_planner_ppt_ppl
    | ChatOpenAI(model="gpt-4-0125-preview", temperature=0)
    # | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    # | ChatOpenAI(model="gpt-4-0613", temperature=0)
    | StrOutputParser()
)

seq_planner_chain_gpt3 = (
    seq_planner_ppt_ppl
    # | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | StrOutputParser()
)


# * output schema of the recursive_sk_generator
class SolutionBehaviortree(BaseModel):
    """the fixed behavior tree"""

    behavior_tree: dict = Field(
        description="the behavior tree that adheres the required format."
    )


behaviortree_file = os.path.join(prompt_dir, "rec_sk_gen/behaviortree.txt")
template_file = os.path.join(prompt_dir, "rec_sk_gen/template.txt")
task_file = os.path.join(prompt_dir, "rec_sk_gen/task.txt")
system_file = os.path.join(prompt_dir, "rec_sk_gen/system.txt")
object_file = os.path.join(prompt_dir, "rec_sk_gen/object.txt")
domain_file = os.path.join(prompt_dir, "rec_sk_gen/domain.txt")
example_file = os.path.join(prompt_dir, "rec_sk_gen/example.txt")
state_file = os.path.join(prompt_dir, "rec_sk_gen/state.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(object_file, "r") as f:
    object_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())

with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())

with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {object}

    {behaviortree}

    {example}

    {template}
    """
)

re_sk_gen_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("object", object_ppt),
        ("behaviortree", behaviortree_ppt),
        ("example", example_ppt),
        ("state", state_ppt),
    ],
)

rec_sk_gen_chain = (
    re_sk_gen_ppt_ppl
    | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | JsonOutputParser()
)


##################################################### * planner
# * output schema of the planner
class Plan(BaseModel):
    """Plan to follow in future"""

    steps: list[str] = Field(
        description="a list of different steps to follow, should be in sorted order"
    )


template_file = os.path.join(prompt_dir, "planner/template.txt")
task_file = os.path.join(prompt_dir, "planner/task.txt")
system_file = os.path.join(prompt_dir, "planner/system.txt")
domain_file = os.path.join(prompt_dir, "planner/domain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {template}
    """
)

planner_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
    ],
)

planner = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_ppt_ppl
)

#########################################################################################
##################################################### * plan_updater
template_file = os.path.join(prompt_dir, "plan_updater/template.txt")
task_file = os.path.join(prompt_dir, "plan_updater/task.txt")
system_file = os.path.join(prompt_dir, "plan_updater/system.txt")
domain_file = os.path.join(prompt_dir, "plan_updater/domain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {template}
    """
)

plan_updater_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
    ],
)

plan_updater = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), plan_updater_ppt_ppl
)

#########################################################################################
# * human instruction helped behavior tree generation
behaviortree_file = os.path.join(prompt_dir, "human_instruction/behaviortree.txt")
template_file = os.path.join(prompt_dir, "human_instruction/template.txt")
task_file = os.path.join(prompt_dir, "human_instruction/task.txt")
system_file = os.path.join(prompt_dir, "human_instruction/system.txt")
state_file = os.path.join(prompt_dir, "human_instruction/state.txt")
domain_file = os.path.join(prompt_dir, "human_instruction/domain.txt")
example_file = os.path.join(prompt_dir, "human_instruction/example.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())

with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())

with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {behaviortree}

    {state}

    {template}
    """
)

human_instruction_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("state", state_ppt),
        # ("example", example_ppt),
        ("behaviortree", behaviortree_ppt),
    ],
)

human_instruction_chain = (
    human_instruction_ppt_ppl
    # | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)
human_instruction_chain_gpt3 = (
    human_instruction_ppt_ppl
    # | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | JsonOutputParser()
)

# * human instruction v2
behaviortree_file = os.path.join(prompt_dir, "human_instruction_v2/behaviortree.txt")
template_file = os.path.join(prompt_dir, "human_instruction_v2/template.txt")
task_file = os.path.join(prompt_dir, "human_instruction_v2/task.txt")
system_file = os.path.join(prompt_dir, "human_instruction_v2/system.txt")
# state_file = os.path.join(prompt_dir, "human_instruction_v2/state.txt")
domain_file = os.path.join(prompt_dir, "human_instruction_v2/domain.txt")
example_file = os.path.join(prompt_dir, "human_instruction_v2/example.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())

# with open(state_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     state_ppt = ppt_tmp.partial(input=f.read())

with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {behaviortree}

    {example}

    {template}
    """
)

human_instruction_ppt_ppl_v2 = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        # ("state", state_ppt),
        ("example", example_ppt),
        ("behaviortree", behaviortree_ppt),
    ],
)

human_instruction_chain_v2 = (
    human_instruction_ppt_ppl_v2
    # | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

#########################################################################################
# * unit tree generation
system_file = os.path.join(prompt_dir, "new/ut_gen/system.txt")
task_file = os.path.join(prompt_dir, "new/ut_gen/task.txt")
domain_file = os.path.join(prompt_dir, "new/ut_gen/new_domain_nl.txt")
behaviortree_file = os.path.join(prompt_dir, "new/ut_gen/new_behaviortree.txt")
template_file = os.path.join(prompt_dir, "new/ut_gen/template.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {behaviortree}

    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

ut_gen_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("behaviortree", behaviortree_ppt),
        ("format_instructions", format_instructions),
    ],
)

rec_ut_generator_chain = (
    ut_gen_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

rec_ut_generator_chain_ft = (
    ut_gen_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    | ChatOpenAI(
        model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
        temperature=0,
    )
    | JsonOutputParser()
)

# ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v3:8z2ZAg2T

rec_ut_generator_chain_gpt3 = (
    ut_gen_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | JsonOutputParser()
)

#########################################################################################
# * state predictor
system_file = os.path.join(prompt_dir, "new/state_est/system.txt")
task_file = os.path.join(prompt_dir, "new/state_est/task.txt")
domain_file = os.path.join(prompt_dir, "new/state_est/new_domain_nl.txt")
state_file = os.path.join(prompt_dir, "new/state_est/state.txt")
output_format_file = os.path.join(prompt_dir, "new/state_est/output_format.txt")
template_file = os.path.join(prompt_dir, "new/state_est/template.txt")
example_file = os.path.join(prompt_dir, "new/state_est/new_example.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {output_format}

    {example}

    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

state_est_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("state", state_ppt),
        ("output_format", output_format_ppt),
        ("format_instructions", format_instructions),
        ("example", example_ppt),
    ],
)


rec_state_predictor_chain = (
    state_est_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo", temperature=0)
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

rec_state_predictor_chain_gpt3 = (
    state_est_ppt_ppl
    | ChatOpenAI(model="gpt-3.5-turbo", temperature=0)
    # | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

#########################################################################################
# * sequence action planner
system_file = os.path.join(prompt_dir, "new/seq_plan/system.txt")
task_file = os.path.join(prompt_dir, "new/seq_plan/task.txt")
domain_file = os.path.join(prompt_dir, "new/seq_plan/new_domain_nl.txt")
state_file = os.path.join(prompt_dir, "new/seq_plan/state.txt")
output_format_file = os.path.join(prompt_dir, "new/seq_plan/output_format.txt")
template_file = os.path.join(prompt_dir, "new/seq_plan/template.txt")
example_file = os.path.join(prompt_dir, "new/seq_plan/new_example.txt")
chain_file = os.path.join(prompt_dir, "new/seq_plan/chain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())
# # * this is for gpt 3.5
# with open(chain_file, "r") as f:
#     chain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {output_format}

    {example}
    
    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

seq_ac_pl_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("state", state_ppt),
        ("output_format", output_format_ppt),
        ("format_instructions", format_instructions),
        ("example", example_ppt),
        # ("chain", chain_ppt),
    ],
)

# seq_ac_pl_chain
rec_sequential_action_planner_chain = (
    seq_ac_pl_ppt_ppl
    | ChatOpenAI(model="gpt-4", temperature=0)
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | JsonOutputParser()
)
rec_sequential_action_planner_chain_gpt3 = (
    seq_ac_pl_ppt_ppl
    # | ChatOpenAI(model="gpt-4", temperature=0)
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | JsonOutputParser()
)

#########################################################################################
# * one_step generation
system_file = os.path.join(prompt_dir, "end_to_end_v3/system.txt")
task_file = os.path.join(prompt_dir, "end_to_end_v3/task.txt")
domain_file = os.path.join(prompt_dir, "end_to_end_v3/domain.txt")
behaviortree_file = os.path.join(prompt_dir, "end_to_end_v3/behaviortree.txt")
example_file = os.path.join(prompt_dir, "end_to_end_v3/example.txt")
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
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {behaviortree}

    {output_format}

    {example}

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
        ("example", example_ppt),
        ("state", state_ppt),
        ("format_instructions", format_instructions),
    ],
)

one_step_chain = (
    e2e_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

one_step_chain_ft = (
    e2e_ppt_ppl
    | ChatOpenAI(
        model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-onestep-v2:9Dq612tc",
        temperature=0,
    )
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    | JsonOutputParser()
)

one_step_chain_gpt3 = (
    e2e_ppt_ppl
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    # | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)


#########################################################################################
# * iterative generation chain

# behaviortree_file = os.path.join(prompt_dir, "rec_sk_gen/behaviortree.txt")
# template_file = os.path.join(prompt_dir, "rec_sk_gen/template.txt")
# task_file = os.path.join(prompt_dir, "rec_sk_gen/task.txt")
# system_file = os.path.join(prompt_dir, "rec_sk_gen/system.txt")
# object_file = os.path.join(prompt_dir, "rec_sk_gen/object.txt")
# domain_file = os.path.join(prompt_dir, "rec_sk_gen/domain.txt")
# example_file = os.path.join(prompt_dir, "rec_sk_gen/example.txt")
# state_file = os.path.join(prompt_dir, "rec_sk_gen/state.txt")
# with open(template_file, "r") as f:
#     template_ppt = PromptTemplate.from_template(f.read())
# with open(task_file, "r") as f:
#     task_ppt = PromptTemplate.from_template(f.read())
# with open(system_file, "r") as f:
#     system_ppt = PromptTemplate.from_template(f.read())
# with open(domain_file, "r") as f:
#     domain_ppt = PromptTemplate.from_template(f.read())
# with open(object_file, "r") as f:
#     object_ppt = PromptTemplate.from_template(f.read())
# with open(behaviortree_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     behaviortree_ppt = ppt_tmp.partial(input=f.read())

# with open(state_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     state_ppt = ppt_tmp.partial(input=f.read())

# with open(example_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     example_ppt = ppt_tmp.partial(input=f.read())
# full_template_ppt = ChatPromptTemplate.from_template(
#     """{system}

#     {task}

#     {domain}

#     {state}

#     {object}

#     {behaviortree}

#     {example}

#     {template}
#     """
# )

# re_sk_gen_ppt_ppl = PipelinePromptTemplate(
#     final_prompt=full_template_ppt,
#     pipeline_prompts=[
#         ("template", template_ppt),
#         ("task", task_ppt),
#         ("system", system_ppt),
#         ("domain", domain_ppt),
#         ("object", object_ppt),
#         ("behaviortree", behaviortree_ppt),
#         ("example", example_ppt),
#         ("state", state_ppt),
#     ],
# )


# re_sk_gen_chain = (
#     re_sk_gen_ppt_ppl
#     | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
#     | JsonOutputParser()
# )

# * iterative generation
system_file = os.path.join(prompt_dir, "iterative_standalone/system.txt")
task_file = os.path.join(prompt_dir, "iterative_standalone/task.txt")
domain_file = os.path.join(prompt_dir, "iterative_standalone/domain.txt")
behaviortree_file = os.path.join(prompt_dir, "iterative_standalone/behaviortree.txt")
example_file = os.path.join(prompt_dir, "iterative_standalone/example.txt")
output_format_file = os.path.join(prompt_dir, "iterative_standalone/output_format.txt")
state_file = os.path.join(prompt_dir, "iterative_standalone/state.txt")
template_file = os.path.join(prompt_dir, "iterative_standalone/template.txt")
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
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {behaviortree}

    {output_format}

    {example}

    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

iterative_generation_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("behaviortree", behaviortree_ppt),
        ("output_format", output_format_ppt),
        ("example", example_ppt),
        ("state", state_ppt),
        ("format_instructions", format_instructions),
    ],
)

iterative_generation_chain = (
    iterative_generation_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

iterative_generation_chain_gpt3 = (
    iterative_generation_ppt_ppl
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    # | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)
