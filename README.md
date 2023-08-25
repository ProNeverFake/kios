<div align="center">
  <img src="/LOGO.jpg" alt="LOGO" width="30%">
</div>

# KIOS --- Knowledge-based Intelligent Operation System

This is the package for robot skill learning and selecting based on ROS2 (based on distro Humble in principle).

The decision making part is realized based on project BehaviorTree.CPP. Code for the communication with MIOS using websocketpp and nlohmann json library is also included.

## Intro

KIOS is developed as a full problem-level robot planning and learning framework based on ROS2. It integrates the high-level planning, which is realized by applying behavior tree mechanism, and the low-level action fine-tuning, which can be achieved by making use of the existing learning algorithms. The functionality of the system is highly decoupled and isolated in the corresponding ROS2 nodes, which composite a cycle like control loop that following the sensing-actuating mode.

The project structure:
- kios
  - kios_cpp
    - messenger
    - tree_node
    - tactician
    - commander
  - kios_py
    - mios_reader
    - mongo_reader
  - kios_interface
  - kios_cli

THIS PART IS STILL UNDER CONSTRUCTION.

> Blackbird: I'm too lazy to write anything here. In fact KIOS is developed for my master thesis and maybe after finishing that I'll finally figure out what this system is really about.

## NEWS

**KNOWN BUGS:**
- MIOS must be launched again after finishing the last tree execution. Otherwise there will be a local error (limit exceeded and context incomplete).
- RCLCPP_INFO macro cannot print element of std::vector (segmentation fault).
- ~~time delay in mios_reader (to be verified). (vielleicht wegen der unrechtzeitigen Nachrichtverhandlung.)~~
- ~~velocity limits are always violated with node Contact in the tree. (10082023)~~
- ~~check: mios task context saving. (14082023)~~
- **[FATAL]** mios cannot build at personal laptop (ubuntu 22.04). 
- ~~ segmentation fault in core: unique ptr in franka --- Poco. (currently built with conan poco 1.11.0) ~~

**DEVELOPER'S PLAN:**
- [x] **ERGENT** use thread safe stack for udp in mios_reader to solve the error.
- [X] **ERGENT** tree udp check mechanism and mios skill udp part.
- [x] **ERGENT** add a udp mechanism to realize skill state sharing between mios and kios.
- [ ] ws_client enable request result bool return 
- [ ] (postponed) add meta node for kios node.

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

kIOS, Short for "Knowledge-based Intelligent Operation System", is a robot skill learing and selecting system developed by BlackBird for his master thesis. The system is built based on ROS2 and should be used along with the mios developed by @Lars.

## Getting Started

BB: skip this part.

### Requirements

- Ubuntu 22.04 recommanded. But Ubuntu 20.04 is also fine (which means you need to install ros2 foxy instead of humble)
- linux Realtime kernal. This is the requirement mios (or more precisely the requirement of robot control frequency).
  ...

### Install

1. Install ROS2 humble (or foxy).
2. Install BehaviorTree.CPP.

- **update:** Now you don't need to do that. It is now a built-in library.

3. Install websocketpp apt package.
4. Install nlohmann apt package.
5. clone the project and build.

```
git clone https://gitlab.com/kopino4-templates/readme-template
```
6. enable global auto-fill

```
pip3 install argcomplete
sudo activate-global-python-argcomplete3
```
7. install conan 1.59.0 

8. install mios (Please use branch "BBbranch").

> BB: There must be something I have forgotten. Feel free to start an issue if you get any error with the project (though I don't think I will check the issues so frequently).

### Usage

This part is still under construction.

> BB: GOOD LUCK.

### System Structure

<div align="center">
  <img src="/system_structure.png" alt="system_structure" width="80%">
</div>

### Running Process

The basic idea is to make the decision making part in kios and the skill execution part in mios in a chained loop. Mios and kios must be synchronized in skill execution phase, which means the start/success/failure phase in kios and mios must be handled in exactly the same time step of the system loop. In this way, the realtime cycle in mios skill execution is reserved and protected by being isolated from the communication part, and the realtime response in kios decision making mechanism is also guaranteed because of the non-stop perception feedback and the accessibility of mios.

<div align="center">
  <img src="/process.png" alt="process" width="80%">
</div>

### Testing

Blackbird: I'll just skip this part for now. 

### Development Log

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
