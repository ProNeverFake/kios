<div align="center">
  <img src="/LOGO.jpg" alt="LOGO" width="30%">
</div>

# KIOS --- Knowledge-based Intelligent Operation System

This is the package for robot skill learning and selecting based on ROS2 (based on distro Humble in principle).

The decision making part is realized based on project BehaviorTree.CPP. Code for the communication with MIOS using websocketpp and nlohmann json library is also included.

## Intro

KIOS is developed as a full problem-level robot planning and learning framework based on ROS2. It integrates the high-level planning, which is realized by applying behavior tree mechanism, and the low-level action fine-tuning, which can be achieved by making use of the existing learning algorithms. The functionality of the system is highly decoupled and isolated in the corresponding ROS2 nodes, which composite a cycle like control loop that following the sensing-actuating mode.

## NEWS

**KNOWN BUGS:**
- Mios limitation violation can randomly happen. (maybe because of the mogen_p2p plugin)
- ~~RCLCPP_INFO macro cannot print element of std::vector(segmentation fault).~~(Not solved yet. This bug is found in ros2 foxy.)
- ~~time delay in mios_reader (to be verified).~~(Solved by FILO message queue).
- ~~velocity limits are always violated with node Contact in the tree. (10082023)~~(Inconsistency in ActionPhase enumerator of kios/mios)
- ~~mios cannot build at personal laptop (ubuntu 22.04).~~
- ~~The reactive sequence should not be used with more than one async action node. (error msg see below)~~

**DEVELOPER'S PLAN:**

- [ ] Add new node Planner for high level planning. 
- [x] Wrap the xml generating code in the context of BT.
- [ ] Apply lifecycle node to tree node etc.

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
* [Conclusion](#conclusion)

## What is KIOS?

KIOS, short for "Knowledge-based Intelligent Operation System", is a robot skill learing and selecting system developed by BlackBird for his master thesis. The system is built based on ROS2 and should be used along with the mios developed by @Lars.

## Getting Started

> BB: No I know in fact you don't want to get started.

### Requirements

- Ubuntu 22.04 is threotically recommanded but not verified yet (In the nearest test the link errors about libresolv appeared and are still not solved, which is a problem popped by conan-installed mongocxx). The system was validated in Ubuntu 20.04 with Ros2 Foxy.

- linux Realtime kernal. This is the requirement mios (or more precisely the requirement of robot control frequency).

> BB: This part is still under construction.

### Install

1. Install ROS2 humble (or foxy).

Currently the system is only verified on Ubuntu 20.04 LTS with Ros2 Foxy. In Ubuntu 22.04 environment the mongocxx (installed by conanfile.txt and is a common dependency for mios and kios) may have problem finding the system dependencies. 

2. ~~Install BehaviorTree.CPP.~~

- **update:** Now you don't need to do that. It is now a built-in library.

3. Install websocketpp.

```bash
sudo apt-get install libwebsocketpp-dev
```

4. Install nlohmann.

5. clone the project and build.

```bash
git clone https://gitlab.com/kopino4-templates/readme-template
```

6. enable global auto-fill (skip this if you do not use CLI of kios)

```bash
pip3 install argcomplete
sudo activate-global-python-argcomplete3
```

7. install conan 1.59.0 (please do not use conan 2)

```bash
pip3 install conan==1.59.0
```

8. install mios (Please use branch "BBbranch").

> BB: Please follow the installing instruction of mios and install all the dependencies needed. Btw is there an installing instruction for mios actually?

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

> BB: ...

### Development Log

- 06.11.2023
  - Added coach, planner, action tuner. Added the necessary interfaces.
  - Built up the architecture surrounding the coach for planning and tuning.
  - Adjusted the communication schema.

  - PSEUDO CODE SHOWS THE BASIC PROCESS OF PLANNING AND TUNING BASED ON BEHAVIOR TREE.

- 25.10.2023
  - KIOS is of version 1.0 now.
  - Removed the deprecated/discarded code.
  - Add Nullspace strategy in mios.
  - Add Tool/Gripper grasp/release atomic skills.
  - Add Tool/Gripper pick/place compound skills.
  - Add unified logger.

- 18.10.2023
  - Add TOOL_GRASP skill.
  **TOOL_RELEASE SKILL**
  **USER STOP STATUS CHECK**

- 16.10.2023
  - Add a behavior tree generator in python (kios_py).
  - Add tool change skills (TOOL_LOAD, TOOL_UNLOAD).
  **The need of grasp when grasping the objects with the tool box.**

- 10.10.2023
  1. All the necessary skills (mps) that are necessary for insertion are all loaded in kios now. Including:
  - joint_move
  - cartesian_move
  - gripper_force
  - gripper_move
  - contact
  - wiggle
  **The need of a compact form for actions**

- 02.10.2023
  1. The base class should be divided into different variances.

- 01.10.2023
  1. The the main frame has been finished. Now mios can receive the request but end without any error message. find the reason.
  2. **Be careful when changing the ActionPhaseContext: only the necessary part should be preserved.**
  3. **TO REMOVE: "action context" should not appear in mios skill request context.**

- 25.09.2023
  1. Added object grounding part in tree_root. Now when the tree is generated by xml, the objects needed by action nodes can be set.
  
  The next step is to change the code in tactician and commander to enable skill commands of the new format.

- 23.09.2023
  1. The service ArchiveActionRequest is realized now.
  2. **The crucial question: how should the "object" be grounded in a object-centric robot system?**

- 22.09.2023
  1. The problem is fixed (reason unclear). 
  TODO: the action archive client on the tree node side and the parameter fetch function in tactician.

- 20.09.2023
  1. bas_alloc problem: when DefaultActionContext member variable, the available memory is not enough for the variable and the program quit. After defining it with ptr the problem still exists and leads to a segmentation fault when calling the ptr.

- *19.09.2023*
  1. Der Class DefaultActionContext wird falsch definiert. Der fehler sieht wie folgende aus:
  ```bash
  terminate called after throwing an instance of 'std::length_error'
  what():  basic_string::_M_create
  ```
  Der Fehler hat noch nicht behandelt worden.

- *05.09.2023*
  1. Removed the insertable entity in action context. Removed the corresponding part in mios (precondition).

- *04.09.2023*
  1. Added node elaborations.
  2. Sorted out the tree generation code in tree_root.cpp. 
  3. Fixed the Bug in commander. Now the program can exit with SIGTERM.

- *03.09.2023*
  1. Fixed the bug in action node switching in the tree. (undefined behavior without bool return.)

- *01.09.2023:*
  1. Added CartesianMove and JointMove into ActionPhase. Added corresponding motion primitives and action nodes. Completed the ActionContext and CommandContext. 
  > (BB: THIS PART IS NOT TESTED YET.)
  2. Updated all the print lines in ws_client and tree_node into spdlog format.
  3. Updated Json command format. Approach, Contact and Wiggle are now demo ActionPhases. More general action phases should be added in the future.

  **WARNNING** BE AWARED THAT YOU FORGET SOMETHING IN ACTIONCONTEXT AND SKILLPARAMETER!!!! 


- *30.08.2023:*
  1. The bug in mongo_reader is fixed. Now tree_node can fetch the object dictionary by sending GetObjectRequest service call to mongo reader.
  2. Add MiosState and SensorState in data types. Added updating methods to them and TaskState. Now update methods can be invoked by passing corresponding msg.  
  3. Completed the condition nodes HasObject and AtPosition. Enabled object check and distance check. Now these condition nodes are strongly related to user-specified actions according to provided action name.
  4. Tested the Fallback and Sequence of BT with current condition and action nodes. Now the TaxInsertion skill in mios can be perfectly reproduced by kios.

- *29.08.2023:*
  1. Changed the msg TaskState into nested msg type with MiosState and SensorState.
  2. Restored all code in branch dump. reset branch BBbranch for debug.

- *28.08.2023:*
  1. Added mongo_reader cpp node. The old python node is discarded from now.
  2. Added mongo_client lib and object master class. Merged Object and Parameters from mios.
  3. Successfully read the objects from mongoDB.
  4. Fixed conan dependency management problem. Now the mongocxx and eigen can be sourced from conan.

- *27.08.2023:*
  1. Added send_and_check method to ws_client. Now commander can handle the response from mios.
  2. Added request handling method in commander. Now commander is enabled to stop mios skill execution when a success is invoked on kios side. 
  3. Added tree phase to switch action request and tree state. Now tactician can conduct tree phase check and stay in the same phase with tree node.

- *26.08.2023:*
  1. BehaviorTree.CPP library changed. In reactivesequence, the error of multiple nodes return running is disabled now.
  2. Validated the sequence action execution. The execution flow of kios is validated.

- *24.08.2023:*
  1. Refactored Poco udp with boost.asio.
  2. Fixed the segmentation fault bug in tactician. ---“啊？”
  3. Added SwitchTreePhase Server. Now nodes can invoke tree phase switch out of the tree_node.

- *23.08.2023:*
  1. Added template class HyperMetaNode. Sorted up the behavior_tree dir and create new condition nodes.
  2. Added RunOnce method in template class.
  3. Moved the udp of mios_reader into a new thread to run receive method. The new udp receiver in python is also thread safe.
  4. Upgraded messenger. Now all the callbacks are in the same mutually exclusive callback group. This should make the R/W process in the publisher and subscriptions thread-safe.
  5. Optimized subscription callbacks with moving assignment.

- *22.08.2023:*
  1. Updated the BBGeneralSkill. Imported kios_utils and kios_communication libs. Changed all data_types into kios_utils types.
  2. Added thread-safe udp sender in kios_communication.
  3. Added necessary tree phase switch parts in BBGeneralSkill.

- *21.08.2023:*
  1. Refactored all the tree node. Moved all basic methods into meta node. Removed all data types and linked the behavior_tree lib to kios_utils.

- *20.08.2023:*
  1. Refactored and upgraded tactician and tree_node. 
  2. Removed ActionContext in behavior_tree lib. All data types are now defined in kios_utils lib.
  3. Changed the communication between tree_node and tactician. Now the action phase switch is detected in tree_node and tactician should be informed with SwitchActionRequest srv. TreeState msg (pub sub) is discarded (turn off) for now.

- *19.08.2023:*
  1. Moved ws_client lib to kios_communication. add thread safe udp lib relying on Poco.
  2. Added new enum TreeState for mios-kios state synchronization. The concept is elaborated in * [Running Process](#running-process)
  3. Added udp socket in tree_node. Add more method for tree phase switch (UNFINISHED).

- *18.08.2023:*
  1. Added Poco. Add new udp receiver in mios_reader.

- *17.08.2023:*
  1. Added thread-safe data type.

- *16.08.2023:*
  1. BUG of velocity limit violation fixed. (flag error in tactician)
  2. Added new CLI: turn_on and turn_off. Realized with ros2xxx.api
  

- *15.08.2023:*
  1. BUG of Action Phase inconsistency fixed.
  2. CLI teach_object test succeeded.

- *14.08.2023:*
  1. Realized mongo_reader in kios_py.
  2. DEBUG: action_name is empty. check task module in mios.
  3. Fixed auto-fill error in CLI.
  4. Added CLI service TeachObjectService in commander. Add CLI_node in kios_cli to enable service call.

- *13.08.2023:*
  1. Added kios_cli for inplementing command line interface. Add a test method "say".
  2. Added mongoDB source file.

- *10.08.2023:*
  1. Context inconsistency with mios fixed. Now a single command execution along with mios is possible.
  2. Bugs in tactician, commander and tree_node are fixed.
  3. New launch file debug_launch.py added for debugging.
  4. Added new library kios_utils for containing common data structure.

- *07.08.2023:*
  1. Changed the project name into KIOS. The old name bt_mios_ros2 is discarded.
  2. Added README.md.

## Contribute

Please make yourself a new branch if you want to contribute.

## License

[BBLAB](https://github.com/ProNeverFake)

## Sources

[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
...

## Conclusion

If you have problems, feel free to ask. I appologize for any inconvenience in your use of this project.
