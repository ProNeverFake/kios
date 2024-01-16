#!/usr/bin/python3 -u
from .ws_client import *
import time


class Task:
    def __init__(self, robot):
        self.skill_names = []
        self.skill_types = []
        self.skill_context = dict()

        self.robot = robot
        self.task_uuid = "INVALID"
        self.t_0 = 0

        self.task_start_response = None
        self.task_wait_response = None

    def add_skill(self, name, skill_type, context):
        self.skill_names.append(name)
        self.skill_types.append(skill_type)
        self.skill_context[name] = context

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
        print(self.skill_context)
        response = start_task(self.robot, "GenericTask", parameters)
        self.task_start_response = response

        self.task_uuid = response["result"]["task_uuid"]

    def wait(self):
        result = wait_for_task(self.robot, self.task_uuid)
        print("Task execution took " + str(time.time() - self.t_0) + " s.")
        self.task_wait_response = result
        return result

    def stop(self):
        result = stop_task(self.robot)
        print("Task execution took " + str(time.time() - self.t_0) + " s.")
        return result
