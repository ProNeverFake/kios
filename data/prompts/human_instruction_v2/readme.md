This is for human in the loop.
Instead of using simulation feedback, use human feedback to modify the tree.

V2: add one example, remove the state

current components:
```python
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
```