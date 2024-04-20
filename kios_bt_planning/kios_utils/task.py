#!/usr/bin/python3 -u
from .ws_client import *
import time

import logging

mios_task_logger = logging.getLogger("mios_task_interface")
mios_task_logger.setLevel(logging.INFO)


class Skill:
    def __init__(self, skill_name=None, skill_type=None, skill_context=None):
        self.skill_name = skill_name
        self.skill_type = skill_type
        self.skill_context = skill_context

    def initialize(self, skill_name, skill_type, skill_context):
        self.skill_name = skill_name
        self.skill_type = skill_type
        self.skill_context = skill_context


class Task:
    def __init__(self, robot, shared_data=None):
        self.skill_names = []
        self.skill_types = []
        self.skill_context = dict()

        self.robot = robot
        self.task_uuid = "INVALID"
        self.t_0 = 0

        self.task_start_response = None
        self.task_wait_response = None
        self.shared_data = shared_data

    def add_skill(self, name, skill_type, context):
        self.skill_names.append(name)
        self.skill_types.append(skill_type)
        self.skill_context[name] = context

    # def add_skill(self, skill: Skill):
    #     self.skill_names.append(skill.skill_name)
    #     self.skill_types.append(skill.skill_type)
    #     self.skill_context[skill.skill_name] = skill.skill_context

    def start(self, queue: bool = False):
        self.t_0 = time.time()
        parameters = {
            "parameters": {
                "skill_names": self.skill_names,
                "skill_types": self.skill_types,
                "as_queue": queue,
            },
            "skills": self.skill_context,
        }

        mios_task_logger.info(f"Starting task with skills: {self.skill_names}")
        response = start_task(self.robot, "GenericTask", parameters)
        self.task_start_response = response

        # TODO YOU SHOULD CHECK THE RESPONSE HERE
        if response is not None:
            if response["result"]["result"] is False:
                mios_task_logger.error(
                    f"Failed to start the skill at mios: {self.skill_names[0]}"
                )
                return None
            else:
                if response["result"]["task_uuid"] is not None:
                    mios_task_logger.info("Task started successfully")
                    self.task_uuid = response["result"]["task_uuid"]
                else:
                    mios_task_logger.error(
                        f"Response includes no UUID for the task {self.skill_names[0]}"
                    )
                    return None
        return response

    def wait(self) -> dict:
        """
        a blocking function to start a observer in mios to monitor the execution of the task.

        Returns:
            mios response in json format
        """
        mios_task_logger.info(f"Wait for skill {self.skill_names[0]} to finish")
        result = wait_for_task(self.robot, self.task_uuid)
        mios_task_logger.info(
            "Task execution took " + str(time.time() - self.t_0) + " s."
        )
        self.task_wait_response = result
        return result

    def stop(self):
        result = stop_task(self.robot)
        mios_task_logger.info("Task stopped!")
        mios_task_logger.info(
            "Task execution took " + str(time.time() - self.t_0) + " s."
        )
        return result

    def interrupt(self):
        result = stop_task(
            self.robot, raise_exception=False, recover=False, empty_queue=False
        )
        mios_task_logger.info("Task interrupted!")
        mios_task_logger.info(str(result))
        return result

    def initialize(self):
        self.shared_data["task_start_response"] = None
        self.task_start_response = None
        self.task_wait_response = None
        self.task_uuid = "INVALID"
        self.t_0 = 0

    def clear_skills(self):
        self.skill_names = []
        self.skill_types = []
        self.skill_context = dict()
