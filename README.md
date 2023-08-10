# KIOS --- Knowledge-based Intelligent Operation System

This is the package for robot skill learning and selecting based on ROS2 (based on distro Humble in principle).

The decision making part is realized based on project BehaviorTree.CPP. Code for the communication with MIOS using websocketpp and nlohmann json library is also included.

## Intro

Blackbird: I'm too lazy to write anything here. In fact KIOS is developed for my master thesis and maybe after finishing that I'll finally figure out what this system is really about.

## NEWS

**DEVELOPER'S PLAN:**
- [ ] ws_client upgrade the log --> spdlog
- [ ] ws_client enable request result bool return 

SEE [DEVELOPMENT LOG](#development-log)

## Contents

* [What is KIOS?](#what-is-KIOS)
* [Getting started](#getting-started)
  * [Requirements](#requirements)
  * [Install](#install)
  * [Usage](#usage)
  * [Used Technologies](#used-technologies)
  * [Testing](#testing)
  * [Development Log](#development-log)
* [Contribute](#contribute)
* [License](#license)
* [Sources](#sources)
* [Conclusion](#conclusion)

## What is KIOS?

kIOS, Short for "Knowledge-based Intelligent Operation System", is a robot skill learing and selecting system developed by BlackBird for his master thesis. The system is built based on ROS2 and should be used along with the mios developed by @Lars.

## Getting Started

BB: Don't need to.

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

6. There must be something I have forgotten. Feel free to start an issue if you get any error with the project (though I don't think I will check the issues so frequently).

### Usage

I'm sorry to inform you that the system is still not finished yet. Before finishing the system I have no time to provide any tutorial of this system. So LEARN BY YOURSELF, just like how you have learned to use mios.

BB: GOOD LUCK.

### Used technologies

- Behavior-tree.
- Websocket communication.
  ...

### Testing

Blackbird: I'll just skip this part. Don't ask me why.

### Development Log

- *07.08.2023:*
  1. Change the project name into KIOS. The old name bt_mios_ros2 is discarded.
  2. Add README.md.

## Contribute

Please make yourself a new branch if you want to contribute.

## License

[BBLAB](https://github.com/ProNeverFake)

## Sources

[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
...

## Conclusion

If you have problems, feel free to ask. I appologize for any inconvenience in your use of this project.
