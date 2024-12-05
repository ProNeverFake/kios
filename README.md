<div align="center">
  <img src="/pic/kios_logo.png" alt="LOGO" width="30%">
</div>

# KIOS --- Knowledge-based Intelligent Operation System

This is a project for robot task planning and execution based on langchain agent, LLM, behavior tree and robot expert skills.

> :star: A conference paper based on this project has been submitted to ICRA 2025. A preprint is available [here](https://arxiv.org/abs/2409.10444).

> :star: A workshop paper based on this project is accepted by ICRA 2024. Check it [here](/workshop.pdf).

> 🎥 The video of human-in-the-loop behavior tree generation is updated. Check it [here](https://www.youtube.com/watch?v=I4f-lSW6qdQ).

> :eyes: The human-in-the-loop workflow tutorial is updated. Please check [here](#something-to-try) for more information.

- About the old version: 
See project [kios_ros2](https://github.com/ProNeverFake/kios_ros2). The old version is developed based on ROS2 and is refactored as this project for several technical reasons.

- About the new version:
The no-ros version, which aims at simplizing the system structure, is now actively developed.
The python package for this project is [kios_bt_planning](/kios_bt_planning).

- About the robot interface:
This project is developed to cooperate with the robot interface [mios](https://gitlab.lrz.de/ki_fabrik_integration/MIRMI-public/mios), which is the skill base developed by the [KI Fabrik](https://kifabrik.mirmi.tum.de/solutions/robot-learning/) team. Mios provides public [docker image](https://hub.docker.com/r/mirmi/mios), which however does not include the necessary modifications for this project (for example, the object grounding process is changed to optional for this project, and skills and kinematic settings for tool-based manipulation are newly developed).
A mios docker image for this project will be packed up and published in the future. Currently, for running the demo, please uncomment the simulation-related code in the script to allow running dummy execution (check [here](#0-enable-the-dummy-execution)). 

You could also may deploy your own methods to generate `robot command` in `mios_task_factory.py` and your own methods to execute the commands in `robot_command.py`. You can also define your own command class. Please search for `MiosCall` and `KiosCall` globally in the project for more details.

## Intro

<div align="center">
  <img src="/pic/headline.png" alt="headline" width="80%">
</div>

KIOS is a LLM & behavior tree-based robot task planning system developed by BlackBird for his master thesis. 
The system is written in python. The idea is to integrate LLMs into the robot task planning system for automatic behavior tree generation and modification.

The LLM is used for generating the task plan in the form of behavior trees based on the provided domain knowledge (prompt engineering or RAG). The APIs for generating, modifying and executing the behavior trees are exposed to the LLM agent. With the feedback from the robot(also the nature language feedbacks from the user), the LLM agent can modify the behavior tree and generate new plans dynamically to finish the robotic assembly tasks.

The usecases are from the siemens robot assembly challenge and the furniture-bench.

## Contents

* [What is KIOS?](#what-is-KIOS)
* [Getting started](#getting-started)
  * [Requirements](#requirements)
  * [Install](#install)
  * [Packages](#packages)
  * [System Structure](#system-structure)
    * [World State](#world-state)
    * [Behavior Tree](#behavior-tree)
    * [Prompt](#prompt)
  * [Something to try](#something-to-try)
  * [Testing](#testing)
  * [Development Log](#development-log)
* [Contribute](#contribute)
* [License](#license)
* [Sources](#sources)

## What is KIOS?

KIOS is a robot intelligent task planning system developed by BlackBird for his master thesis. Some of the key features are:

- natural language understanding and interacting
- assembly task planning based on state and knowledge
- behavior tree generation, modification and execution

## Getting Started

### Requirements

- Ubuntu 20.04 LTS (verified)
- conan 1.59.0 (the requirements if you use MIOS, otherwise unnecessary)
- linux Realtime kernal(the requirement of franka panda control interface, ignore this if you do not play any robot action). For walkthrough please check [here](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).

### Install

It is highly recommended to use a virtual environment for the project.

```bash
conda create -n kios python=3.10
conda activate kios
```

1. Install dependency packages.

```bash
pip3 install -r requirements.txt
sudo apt-get install graphviz
# install the package kios_bt_planning
cd kios_bt_planning
pip3 install -e .
# for testing the code with livescript it is recommended to install ipython in this virtual environment.
conda install ipython
```

2. (This is only for world state visualization. Skip this if you do not need it.) install neo4j.

The application can be downloaded from [here](https://github.com/neo4j/neo4j-python-driver).

After setting up the neo4j server, please change the autherization information in `kios_bt_planning/kios_world/neo4j_interface.py`.


3. (This is only for MIRMI users who want to play real robot actions with MIOS) Set up the mios (branch = kios) and the franka robot.

> BB: For MIRMI users, check the project [mios](https://gitlab.lrz.de/ki_fabrik_integration/MIRMI-public/mios) for more information. The docker image's name is "mirmi/mios", but is not compatible with this project. The skills necessary for the robot manipulation in kios are still being actively developed. A new docker image will be released as soon as possible. 

4. Set up your openai api-key.

If you want to use the openai gpt models as your LLM, please set up your openai api-key globally according to this [link](https://platform.openai.com/docs/quickstart?context=python).

```bash
cd ~/
nano ./.bashrc
# just put this line into your .bashrc file in your home directory.
export OPENAI_API_KEY='xxxxxx'
# you need to start a new terminal to make it take effect.
```
> BB: Protect your api-key carefully and prevent any secret leakage.

5. (This is only for langchain monitor and data collection) Set up langsmith.

If you want to use langsmith to minitor the LLM queries, take a look at this [link](https://docs.smith.langchain.com/tracing) to set the api-key.

You can also use something else like [langfuse](https://github.com/langfuse/langfuse) to monitor the LLM queries.

6. (This is for MIOS operation. Skip this if you do not play real robot actions) Setup MongoDB.

Please check this [link](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/) to install the mongoDB. You should also start the mongoDB service after the installation!

### Packages
- data:
  - prompt: the prompt engineering files for the LLMs.
  - world_definition: the definitions of the actions in pddl style.
  - router_utterances: the utterances for the semantic router.
  - 
- experiments: the experiment files of different problems.
  - chair...
  - gearset...
    - scene
    - domain
    - problem
  - gearset1... (for development and testing)
    - scene
    - domain
    - problem
  - demo (for demo)
    - human_in_the_loop_sync.py
    - iterative_generation_sync.py
    - one_step_generation_sync.py
    - recursive_generation_sync.py
    - world_state.json
    - scene.json
    - ...
- kios_bt_planning
  - kios_agent: the agents for task planning and behavior tree generating
    - kios_llm_bt: prompt engineering files for end-to-end behavior tree generating
  - kios_bt: modules for basic behavior tree functionality.
    - Behavior nodes (actions and conditions)
    - The factory class for generating behavior trees
    - Behavior tree json interface
    - Mios asynchronization module
    - ...
  - kios_domain: domain knowledge written in pddl with unified-planning
    - pddl python interfaces.
    - domain knowledge definitions.
    - ...
  - kios_planner: discarded now
  - kios_robot: robot modules for real-world robot manipulation
    - kios_vision
    - robot_interface: interface methods to execute the actions in behavior trees.
    - robot_proprioceptor: class for interacting with the robot (get/set states).
    - mios_task_factory: task factory for miosskill, mioscall and kioscall.
    - mios_async: asynchronization module for robot_command
    - robot_command: the command (list of calls/skills) for the robot.
  - kios_scene: modules for the task scene model
    - mongodb_interface
    - scene_factory
    - (scene-world_linker)
  - kios_world: modules for the world model
    - world_interface: interfaces for query/update the world state.
    - graph_interface: interfaces for interacting with inner world graph.
    - neo4j_interface: interfaces for neo4j database.
  - kios_utils: utility modules
  - tests: test files for the modules above.

### System Structure

<div align="center">
  <img src="/pic/CONCEPT.png" alt="The Concept" width="90%">
</div>

#### World State

The world state in the framework is modeled with a dictionary-like structure and organized in a JSON object.
Using JSON files as world representation leverages the rich JSON-related data in the pre-training phase.

<div align="center">
  <img src="/pic/world_state_modeling.png" alt="world state" width="45%">
  <img src="/pic/world_model_vis.png" alt="world state vis" width="45%">
</div>

The key values in the world state are explained below:

- Objects

  A list of the objects in the world, including their names and the properties they have.


- Properties

  Properties are typically unary state variables that indicate object affordances and availability. These properties can change during task execution. For example, `is_available(tool1)` indicates that the tool is available for task execution, and this status may change when the tool is occupied.


- Constraints

  A list of constraints in the world that the user defines, including the constraint name and the two objects affected by the constraint. A constraint can be either a geometry constraint between two objects (e.g., a cylinder can be inserted into a round hole) or a form of user knowledge (e.g., a clamp-gripper can be used to manipulate a large-sized gear). Constraints are pre-defined knowledge and cannot be changed during the task process.

- Relations

  A list of relations in the world, including the relation name and the two objects involved. Most relations are geometry (e.g., a peg is inserted into a hole), while others are semantic (e.g., the hand is holding a clamp-gripper). The task target can be defined as relations that are changeable during the plan execution.


#### Behavior Tree

The BTs generated and utilized in the system are in JSON format.

<div align="center">
  <img src="/pic/bt_modeling.png" alt="behavior tree" width="60%">
</div>

<div align="center">
  <img src="/pic/bt_modeling_vis.png" alt="behaivor tree vis" width="80%">
</div>

In the JSON file of BTs, each node has a *summary* that provides a brief description and a *name* that reflects the node type and employs domain knowledge definitions of the name form. There are several node types, including *selector*, *sequence*, *condition* (which is further classified into *target* and *precondition*), and *action*. The *selector* and *sequence* nodes control the tick flow of BTs and contain a list of subsequent nodes called *children*. Condition nodes labeled as *target* are typically children of *selectors*, while those categorized as *preconditions* are found as children of *sequences*. It is crucial that all nodes align with their corresponding actions or predicates, as defined within the domain knowledge. Control flow nodes in BTs have no memory, which means each tick starts at the root and traverses through all nodes anew, disregarding previous states of the control flow nodes. The basic structure of a unit subtree includes a root *selector* node, a *target* condition node as its first child to verify target satisfaction, followed by a *sequence* node aimed at fulfilling the target condition. The *sequence* node starts with several *precondition* nodes that validate necessary conditions before executing an action, and it concludes with an *action* node. This *action* node is designed to achieve effects that satisfy the *target* node in the upper-level *selector*, ensuring the subtree's functional coherence and goal-directed behavior.


#### Prompt

Here is an overview of the prompt structure used in the project:

<div align="center">
  <img src="/pic/prompt.png" alt="behavior tree" width="80%">
</div>


### Something to try

#### 0. Enable the dummy execution

The docker image of mios is currently not available. You can enable the dummy execution by uncommenting the code in the demo script, which allows the execution to be simulated and the effects of the actions will be applied to the world state after the execution.

The code for dummy execution is (you can find it with ctrl+f):

```python
return behavior_tree_simulation_step(state)
```

Uncommenting this line will call the simulation node of the langgraph to simulate the execution of the behavior tree and skip the interaction with the robot interface.

#### 1. Runtime script for robot commands (For MIRMI users)

The scripts `runtime_script.py` (just search them in the project) are live scripts for modifying mios memory, teaching mios objects (check mios documentation to understand what are the objects in mios), quick environment setup and robot command testing.

Import it with ipython so you can call the function at runtime:

```bash
# in the virtual environment
# change to its dir (gearset1 for example)
cd experiments/gearset1

# you need ipython installed in conda
ipython -i runtime_script.py

# run the commands...
```

Please check the script for more information about the functions.

#### 2. Human-in-the-loop behavior tree generation (For all users)

The human-in-the-loop behavior tree generation is a process for generating behavior trees iteratively with the help of human feedback. User input is first passed to the assembly planner, which makes a high-level assembly plan including several product-concentrated assembly steps. Then the first step is passed to the sequential planner to generate an action sequence in natural language, which helps to generate the corresponding behavior tree in the behavior tree generator. The behavior tree is a mid-level plan about robot actions and condition checking. The user is asked to provide feedback to help improve or correct the beahvior tree in natural language. The feedback is then used to modify the behavior tree and generate a new plan(tree). The process is repeated until the user is satisfied with the behavior tree. Then the behavior tree is executed by the robot, which calls the robot interface to run low-level motion primitives or skills. The execution will stop when the tree gets a feedback and the user will be asked to provide feedback again. After successfully finishing the task, the plan updater will update the plan in the assembly planner and the process will be repeated for the next step until the whole assembly task is finished.

Following is the workflow for human-in-the-loop behavior tree generation:

<div align="center">
  <img src="/pic/human_in_the_loop.png" alt="human-in-the-loop generation" width="90%">
</div>

User input: assembly instructions (natural language).

User_feedback: suggestions for the behavior tree (natural language).

```bash
# in the virtual environment
cd experiments/demo
python human_in_the_loop_sync.py
```

It is strongly recommended to watch the video [here](https://www.youtube.com/watch?v=I4f-lSW6qdQ) to understand the workflow. You can also try the same inputs that are shown in the video.

### Testing (with livescript and MIOS)

For module testing please check the test folder in `kios_bt_planning`.

To use ipython for debug, install ipython in your environment (otherwise will use the system-wide interpreter):

```bash
conda install ipython
```

### Development Log

...

## Contribute

You are welcome to contribute to the project by starting an issue or making a pull request.

## Citation

```
@inproceedings{Ao2024workshop-LLM-BT,
 author = {Ao, Jicong and Wu, Yansong and Wu, Fan and Haddadin, Sami},
 booktitle = {ICRA 2024 Workshop Exploring Role Allocation in Human-Robot Co-Manipulation},
 title = {Behavior Tree Generation using Large Language Models for Sequential
Manipulation Planning with Human Instructions and Feedback},
 year = {2024}
}
```

## License

MIT License

## Sources

- [semantic-router](https://github.com/aurelio-labs/semantic-router)
- [huggingface](https://huggingface.co/)
- [langchain](https://python.langchain.com/docs/get_started/introduction)
- [langsmith](https://docs.smith.langchain.com/tracing)
- [llama_cpp_python](https://github.com/abetlen/llama-cpp-python)
- [llama_factory](https://github.com/hiyouga/LLaMA-Factory)
- [Mistral](https://mistral.ai/technology/#models)
- [llama2](https://llama.meta.com/llama2)
- [ChatGPT-Robot-Manipulation-Prompts](https://github.com/microsoft/ChatGPT-Robot-Manipulation-Prompts)
- [furniture-bench](https://github.com/clvrai/furniture-bench)
- [mios_py_interface](https://gitlab.lrz.de/bblab/mios_py_interface)
- [unified-planning](https://github.com/aiplan4eu/unified-planning)
- [mios](https://gitlab.lrz.de/ki_fabrik_integration/MIRMI-public/mios)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [py_trees](https://github.com/splintered-reality/py_trees)
- [neo4j](https://github.com/neo4j/neo4j-python-driver)
- [websocketpp](https://github.com/zaphoyd/websocketpp)
- [ros2](https://docs.ros.org/en/foxy/index.html)
