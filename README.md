<div align="center">
  <img src="/LOGO.jpg" alt="LOGO" width="30%">
</div>

# KIOS --- Knowledge-based Intelligent Operation System

This is the package for robot skill learning and selecting based on ROS2 (based on distro Humble in principle).

- About the old version: 
The behavior tree executor is implemented based on project BehaviorTree.CPP. See `kios_cpp`.
Some of the components are implemented in python. See `kios_py`.

> BB: For the old version, please check `old_readme.md`. The old version will be archived and will not be updated anymore. 

- About the new version:
The no-ros version, which aims at simplizing the system structure, is now actively developed.
For this part see `kios_bt_planning`.

## Intro

KIOS is a robot task planning system developed by BlackBird for his master thesis. The system is currently under development and is not ready for use. 

The system is mainly written in python. The idea is to integrate the LLM (large language model) into the robot task planning system for automatic behaviortree generation and modification.

The LLM is used for generating the task plan in the form of behavior trees based on the provided domain knowledge (prompt engineering or RAG). The APIs for generating, modifying and executing the behavior trees are exposed to the LLM agent. With the feedbacks from robot sensors and the cameras (also the nature language feedbacks from the user), the LLM agent can modify the behavior tree and generate a new plan for the robot to finish the robotic assembly tasks.

The usecases are from the siemens robot assembly challenge and the furniture-bench.

## Contents

* [What is KIOS?](#what-is-KIOS)
* [Getting started](#getting-started)
  * [Requirements](#requirements)
  * [Install](#install)
  * [Packages](#packages)
  * [Usage](#usage)
  * [System Structure](#system-structure)
  * [Running Process](#running-process)
  * [Testing](#testing)
  * [Development Log](#development-log)
* [Contribute](#contribute)
* [License](#license)
* [Sources](#sources)
* [More](#more)

## What is KIOS?


## Getting Started

### Requirements

- Ubuntu 20.04 LTS

- linux Realtime kernal. This is the requirement of robot control interface (1000Hz control loop).

> BB: This part is still under construction.

### Install

It is highly recommended to use a virtual environment for the project.

```bash

conda create -n kios python=3.9

conda activate kios

```

1. Install dependency packages.

```bash
pip install -r requirements.txt

sudo apt-get install graphviz

# install the package kios_bt_planning
cd kios_bt_planning
pip install -e .
```

If you use llama, setup the environment following the instructions [here](https://github.com/facebookresearch/llama).

Don't forget to install the packages in the local environment your created above.

2. (Skip this if you do not need visualization) install neo4j.

The application can be downloaded from [here](https://github.com/neo4j/neo4j-python-driver).

After setting up the neo4j server, please change in the neo4j interface in `kios_bt_planning/kios_world` to your pw.


3. Set up the mios (branch = kios) and the robot.

> BB: Go to check the project [mios](https://gitlab.lrz.de/ki_fabrik_integration/MIRMI-public/mios) for more information. GL.


> BB: The project is still under development. Please feel free to start an issue if you have any question or suggestion. 

### Packages

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
    - robot_interface: interface methods to execute the actions in behavior trees.
    - robot_proprioceptor: class for getting robot states
    - robot_status: class for keeping the robot status.
    - robot_actuator: primitive actions for robot manipulation.
    - robot_skill_engine: compound actions for robot manipulation.
  - kios_world: modules for the world model
    - world_interface: interfaces for query/update the world state.
    - graph_interface: interfaces for interacting with inner world graph.
    - neo4j_interface: interfaces for neo4j database.
  - kios_utils: utility modules
  - tests: test files for the modules above.
(old version packages)
- kios_cpp: the behavior tree executor implemented in c++. discarded now.
- kios_py: the behavior tree executor implemented in python. discarded now.
- kios_cli: the command line interface for the project. discarded now.

### Usage

> BB: Now the functionality of the project is still under development. I'm now trying to deploy the LLM agent as soon as possible, which should be transferred to this project as a submodule in the next step. 

> BB: For now please try out the test files in `kios_bt_planning`.

### System Structure

<div align="center">
  <img src="/TRI.png" alt="The Concept" width="90%">
</div>

### Running Process

...

### Testing

For module testing please check the test folder in `kios_bt_planning`.

To use ipython for debug, install ipython in your environment (otherwise will use the system-wide interpreter):

```bash
conda install ipython
```

### Development Log

For the old version see [old_dev_log.md](old_dev_log.md)

## Contribute

The project is still under development. You are welcome to contribute to the project by starting an issue or making a pull request. It is strongly recommended to start a new branch for module contribution.

## License

MIT License

## Sources

[ChatGPT-Robot-Manipulation-Prompts](https://github.com/microsoft/ChatGPT-Robot-Manipulation-Prompts)

[furniture-bench](https://github.com/clvrai/furniture-bench)

[mios_py_interface](https://gitlab.lrz.de/bblab/mios_py_interface)

[unified-planning](https://github.com/aiplan4eu/unified-planning)

[mios](https://gitlab.lrz.de/ki_fabrik_integration/MIRMI-public/mios)

[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)

[py_trees](https://github.com/splintered-reality/py_trees)

[neo4j](https://github.com/neo4j/neo4j-python-driver)

[websocketpp](https://github.com/zaphoyd/websocketpp)

[ros2](https://docs.ros.org/en/foxy/index.html)

...

## More

The project is still under development. Please feel free to start an issue if you have any question or suggestion.

The tutorial for the new version is still under construction. It will be released as soon as possible (maybe sometime around 15.04.2024, which is the deadline of my master thesis).
