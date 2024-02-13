from typing import List, Any
import time
import json
from tabulate import tabulate
from collections import deque
import asyncio
import logging

from kios_utils.task import *
from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import MiosInterfaceResponse, MiosTaskResult
from kios_robot.data_types import MiosSkill, MiosCall, KiosCall
from kios_robot.data_types import TaskScene


class AsyncRobotCommand:
    robot_address: str = None
    robot_port: int = None
    shared_data: Any = None

    robot_interface: RobotInterface = None

    task_scene: TaskScene = None

    tasks: deque[MiosSkill | MiosCall | KiosCall] = []

    emergency_flag: bool
    error_flag: bool
    current_task: MiosSkill | MiosCall | KiosCall

    def __init__(
        self,
        robot_address: str,
        robot_port: int,
        shared_data: Any,
        task_scene: TaskScene,
        robot_interface: RobotInterface,  # ! not used yet
    ):
        if robot_address is not None:
            self.robot_address = robot_address
        else:
            raise Exception("robot_address is not set")

        if robot_port is not None:
            self.robot_port = robot_port
        else:
            raise Exception("robot_port is not set")

        if shared_data is not None:
            self.shared_data = shared_data
        else:
            print("warning: robot command shared_data is not set.")

        if task_scene is not None:
            self.task_scene = task_scene
        else:
            # raise Exception("robot_scene is not set")
            print("warning: robot command task_scene is not set.")

        if robot_interface is not None:
            self.robot_interface = robot_interface
        else:
            raise Exception("robot_interface is not set")

        self.tasks = deque()
        self.emergency_flag = False
        self.error_flag = False
        self.current_task = None

    def initialize(self):
        pass
        # self.task_list = [] ! dont do this. the task list should be kept.
        # * currently the command response is just a boolean.
        # * can be extended to a more complex data type.

    def interrupt(self):
        # dirty.
        payload = {
            "raise_exception": False,
            "recover": False,
            "empty_queue": False,
        }
        call_method(self.robot_address, self.robot_port, "stop_task", payload=payload)

    async def add_task(
        self, task: MiosSkill | MiosCall | KiosCall, is_emergent: bool = False
    ):
        """Add a task to the task list. If it's emergent, clear current tasks and set the emergency flag."""
        if is_emergent:
            self.emergency_flag = True
            # self.tasks.clear()  # discard or postpone existing tasks
            self.tasks.appendleft(task)
        else:
            self.tasks.append(task)

        # If the robot is idle or an emergency task is added, start task processing
        if not self.current_task or is_emergent:
            await self.process_tasks()

    async def process_tasks(self):
        """Process tasks one by one, giving priority to emergent tasks."""
        while self.tasks:
            if self.emergency_flag:
                self.emergency_flag = (
                    False  # Reset the flag after accepting the emergent task
                )
            self.current_task = self.tasks.popleft()
            await self.execute_task(self.current_task)

            self.current_task = None

    async def execute_task(self, task) -> None:
        """Simulate task execution."""
        print(f"Executing task: {str(task)}")

        try:
            if isinstance(task, MiosSkill):  # * use general task
                mios_task = Task(self.robot_address)
                mios_task.add_skill(
                    task.skill_name,
                    task.skill_type,
                    task.skill_parameters,
                )
                start_result = mios_task.start()

                logging.info(f"Starting mios skill: {str(task)}")
                mios_start_response = MiosInterfaceResponse.from_json(
                    start_result["result"]
                )
                logging.info("\033[92mMios replied: ")
                logging.info(mios_start_response)
                logging.info("\033[0m")

                # * check if the task is started successfully
                if mios_start_response.has_finished == False:
                    logging.error(f"Mios skill {str(task)} failed to start!")
                    raise Exception("mios skill failed to start!")

                result = mios_task.wait()
                mios_response = MiosInterfaceResponse.from_json(result["result"])
                logging.info("\033[92mMios replied: ")
                logging.info(mios_response)
                logging.info("\033[0m")  # Reset color to default

                # * if the task is not finished successfully, return a false
                if mios_response.has_finished == False:
                    logging.error(f"Mios skill {str(task)} failed to finish!")
                    raise Exception("mios skill failed to finish!")

                if mios_response.task_result.has_succeeded == False:
                    logging.error(f"Mios skill {str(task)} failed to succeed!")
                    raise Exception("mios skill failed to succeed!")

            elif isinstance(task, MiosCall):  # * use general call
                result = call_method(
                    self.robot_address,
                    self.robot_port,
                    task.method_name,
                    task.method_payload,
                )
                if result is None:
                    logging.error(
                        f"Mios call {str(task)} failed! Please check mios for more debug info."
                    )
                    raise Exception("Mios call failed!")

                mios_response = MiosInterfaceResponse.from_json(result["result"])
                # print("Result: " + str(result))
                logging.info("\033[92mMios replied: ")
                logging.info(mios_response)
                logging.info("\033[0m")  # Reset color to default
                if mios_response.has_finished == False:
                    logging.error(f"Mios call {str(task)} failed!")
                    raise Exception("Mios call failed!")

            elif isinstance(task, KiosCall):  # * use general call
                result_bool = task.method(*task.args)
                if result_bool != True:
                    logging.error(f"Kios call {str(task)} failed!")
                    raise Exception("Kios call failed!")

            else:
                raise Exception("Unknown task type: {}".format(task))
        except Exception as e:
            logging.warning(f"recovering robot from error: {str(e)}")
            await self.recover_robot()
            logging.warning(f"redo task: {str(task)}")
            await self.execute_task(task)

        logging.info(f"Task finished: {str(task)}")

    async def recover_robot(self):
        """Recover the robot to a safe state before executing an emergent task."""
        print("Recovering robot...")
        self.interrupt()
        # more methods to recover the robot

    async def add_tasks(
        self,
        new_tasks: List[MiosSkill | MiosCall | KiosCall],
        is_emergent: bool = False,
    ):
        """Add a task to the task list. If it's emergent, clear current tasks and set the emergency flag."""
        if is_emergent:
            self.emergency_flag = True
            # self.tasks.clear()  # discard or postpone existing tasks
            self.tasks.extendleft(new_tasks)
        else:
            self.tasks.extend(new_tasks)

        # If the robot is idle or an emergency task is added, start task processing
        if not self.current_task or is_emergent:
            await self.process_tasks()

    def prepend_task(self, task: MiosSkill | MiosCall | KiosCall):
        self.tasks.appendleft(task)

    def prepend_tasks(self, tasks: List[MiosSkill | MiosCall | KiosCall]):
        self.tasks.extendleft(tasks)

    def clear_tasks(self):
        self.tasks.clear()

    def show_tasks(self):
        task_table = []
        for task in self.task_list:
            task_type = type(task).__name__  # mios_skill, mios_call, kios_call
            task_name = str(task)
            task_table.append([task_type, task_name])

        print(tabulate(task_table, headers=["Task Type", "Task Name"], tablefmt="grid"))

        # pass
