<div align="center">
  <img src="/LOGO.jpg" alt="LOGO" width="30%">
</div>

# KIOS --- Knowledge-based Intelligent Operation System

This is the package for robot skill learning and selecting based on ROS2 (based on distro Humble in principle).

## Intro

KIOS is developed as a full-level robot planning and learning framework based on ROS2. It integrates the high-level planning, which is realized by applying behavior tree mechanism, and the low-level action fine-tuning, which can be achieved by making use of the existing learning algorithms. The functionality of the system is highly decoupled and isolated in the corresponding ROS2 nodes, which composite a cycle like control loop that following the sensing-actuating mode.

## NEWS

SEE [DEVELOPMENT LOG](#development-log)

## Contents

* [What is KIOS?](#what-is-KIOS)
* [Getting started](#getting-started)
  * [Requirements](#requirements)
  * [Install](#install)
  * [Usage](#usage)
  * [Used Technologies](#used-technologies)
  * [System Structure](#system-structure)
  * [Running Process](#running-process)
  * [Testing](#testing)
  * [Development Log](#development-log)
* [Contribute](#contribute)
* [License](#license)
* [Sources](#sources)
* [More](#more)

## What is KIOS?

KIOS, short for "Knowledge-based Intelligent Operation System", is a robot skill learing and selecting system developed by BlackBird for his master thesis. The system is built based on ROS2 and should be used along with the mios developed by @Lars.

## Getting Started

> BB: GLHF

### Requirements

- Ubuntu 20.04 is a verified system version for this project.

> BB: (In the nearest test the link errors about libresolv appeared and are still not solved, which is a problem popped by conan-installed mongocxx). The system was validated in Ubuntu 20.04 with Ros2 Foxy.

- linux Realtime kernal. This is the requirement mios (or more precisely the requirement of robot control frequency).

> BB: This part is still under construction.

### Install

1. Install ROS2 foxy.

Currently the system is only verified on Ubuntu 20.04 LTS with Ros2 Foxy. In Ubuntu 22.04 environment the mongocxx (installed by conanfile.txt and is a common dependency for mios and kios) may have problem finding the system dependencies. 

2. ~~Install BehaviorTree.CPP.~~

- **update:** Now you don't need to do that. It is now a built-in library.

3. Install websocketpp.

```bash
sudo apt-get install libwebsocketpp-dev
```

4. Install nlohmann.

5. clone the project and build.

6. enable global auto-fill (Skip this if you do not use CLI of kios)

```bash
pip3 install argcomplete
sudo activate-global-python-argcomplete3
```

7. install conan 1.59.0 (Please do not use conan 2)

```bash
pip3 install conan==1.59.0
```

8. install mios (Please use branch "kios").

> BB: Please follow the installing instruction of mios and install all the dependencies needed. This project is better than mios, because it has a readme file at least.

9. install spdlog.

10. install fmt

```bash
sudo apt install libfmt-dev
```

> BB: There must be something I have forgotten. Feel free to start an issue if you get any error with the project (though I don't think I will check the issues so frequently).

### Usage

The enter point is ...

> BB: Currently coach is taken as the enter point of the program. It should conduct the process in the pseudo code and call the action/service provided by other nodes for the needed functionality.

### System Structure

The overall system structure is shown below.

<div align="center">
  <img src="/system_structure.png" alt="system_structure" width="80%">
</div>

The system consists of a couple of nodes, in which different functionality is decoupled from the main goal and realized independently. 

The project structure:

- kios
  - kios_cpp
    - messenger
    - tree_node
    - tactician
    - commander
    - mongo_reader
  - kios_py
    - bota_sens
    - mios_reader
    - planner
    - skill_tuner
    - coach
  - kios_interface
    - action
      - MakePlan
    - msg
      - MiosState
      - NodeArchive
      - SensorState
      - TaskState
      - TreeState
      - BotaSens
    - srv
      - ArchiveActionRequest
      - CommandRequest
      - GetObjectRequest
      - SwitchActionRequest
      - SwitchTreePhaseRequest
      - TuneSkillRequest
      - TeachObjectRequest
  - kios_cli
    - CLI node

The functions of the nodes are explained explicitly below.

---

##### **mios_reader**

The node **mios_reader** publishes the realtime sensing data from mios.

- Written in python.
- Has a udp receiver member object which receives the packages from the telemetry udp sender in mios.
- The entities sent by mios telemetry is registered in node **commander**.
- Publish the sensing data to topic `mios_state_topic` with msg `MiosState.msg`.
- Has a user-defined package loss tolerance. Power off if it is exceeded (timeout).

For developer:

- The messages(data) are transfered "as they are". They should be restored to the original format at the endpoint that use them. 

---

##### **sensor_reader**

The node **sensor_reader** publishes the realtime sensing data from the sensors.

- Not implemented yet since there is no sensor deployed on my robot.
- Publish the data to topic `sensor_state_topic` with msg `SensorState.msg`.

---

##### **messenger**

The node **messenger** subscribes all the sensing data topics and assemble them with a nested msg, then publish it.

- Subscibe the topic `mios_state_topic` and `sensor_state_topic`.
- Publish to topic `task_state_topic` with msg `TaskState.msg`, which is a msg type nested with `MiosState.msg` and `SensorState.msg`.

For developer:

- The subscribers and publishers are put in a MutuallyExclusiveCallbackGroup and the node is executed by a single-thread executor. This, though may affect the efficiency, can avoid possible data race. Deploy mutex instead if you need higher transfer frequency.

---

##### **tree_node**

The node **tree_node** manages the life cycle of the behavior tree. It subscribes the robot state, determines the next action and asks the node **tactician** for constructing the action.

- Subscribe the topic `task_state_topic`.
- Request action switch by service `switch_action_service` with `SwitchAcitionRequest.srv`.
- Determine the next action by "ticking" the `tree_root`.
- Synchronize the tree phase by receiving state feedback from mios with a udp receiver member object.
- Update the object list with service `get_object_service` with `GetObjectRequest.srv`.

...

---

##### **tactician**

The node **tactician** construct the action with the corresponding context. It receives the switch action request from the node **tree_node** and generate the action context, then asks the node **commander** to send the command.

- member object: paramclerk, which read, write the action parameters from/into a json file in workspace directory.
- Provide the service `switch_action_service` with `SwitchAcitionRequest.srv`.
- fetch the action node's parameters from paramclerk by calling `generate_command_context()`.

For developer:

- The skill currently used in mios is NOT BBGeneralSkill ANYMORE. The action nodes in the behavior_tree library should all have their own corresponding skill (only one) in mios. Multiple action nodes in kios can be mapped to the same skill in mios (e.g. the tool_load and tool_unload, because the only difference is to open or close the gripper.).

- For skills available please see kios_skill in mios (BBbranch)

---

##### **commander**

The node **commander** manages the websocket connection with mios Port. It receives the command request from the node **tactician** and send it to mios websocket server.

- Provide the service `command_request_service` with `CommandRequest.srv`.
- Can `send`, `send_and_wait`, `send_and_check`.
- Provide CLI service `teach_object_cli` with `TeachObjectService.srv`.

---

##### **mongo_reader**

The node **mongo_reader** manages the communication between kios and mongoDB. It reads the objects set in mongoDB with mongoDB client.

- Provide the service `get_object_service` with `GetObjectRequest.srv`.

---

##### **planner** (UNDER CONSTRUCTION)

The node **planner** make plans for robot tasks, in which the expanding BT techniques should be applied. It expands the behavior tree, validates it, transforms it into xml format (dumps it in string format) and send the result back.

--- 

##### **skill_tuner** (UNDER CONSTRUCTION)

The node for tuning the parameters of the skills in the plan (i.e. the action nodes in the behavior tree).

- Provide the service `tune_skill_service` with `TuneSkillRequest.srv`.

- Publish ...

--- 

##### **bota_sens** (UNDER CONSTRUCTION)

The sensor node for Bota SensOne F/T sensor. 

- Publish ...

--- 

##### **coach** (UNDER CONSTRUCTION)

The "agent". The node "coach" is a client for calling all possible service provided above.

...


### Running Process

The basic idea is to make the decision making part in kios and the skill execution part in mios in a chained loop. Mios and kios must be synchronized in skill execution phase, which means the start/success/failure phase in kios and mios must be handled in exactly the same time step of the system loop. In this way, the realtime cycle in mios skill execution is reserved and protected by being isolated from the communication part, and the realtime response in kios decision making mechanism is also guaranteed because of the non-stop perception feedback and the accessibility of mios.

<div align="center">
  <img src="/process.png" alt="process" width="80%">
</div>

### Testing

The code are currently in debug mode. The test is conducted in the following environment:

- Ubuntu 20.04 LTS
- ROS2 Foxy
- mios (branch: kios)
- BehaviorTree.CPP 4.x
- conan 1.59.0 (important)



### Development Log

For the old version see [old_dev_log.md](old_dev_log.md)

## Contribute

The project is still under development. You are welcome to contribute to the project by starting an issue or making a pull request. It is strongly recommended to start a new branch for module contribution.

## License

MIT License

## Sources
[mios_py_interface](https://gitlab.lrz.de/bblab/mios_py_interface)
[mios](https://gitlab.lrz.de/ki_fabrik_integration/MIRMI-public/mios)
[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
[websocketpp](https://github.com/zaphoyd/websocketpp)
[ros2](https://docs.ros.org/en/foxy/index.html)

...

## More

The project is still under development. Please feel free to start an issue if you have any question or suggestion.

The tutorial for the new version is still under construction. It will be released as soon as possible (maybe sometime around 15.04.2024, which is the deadline of my master thesis).
