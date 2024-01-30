from kios_utils.task import *
from typing import List, Any
import time
import json

from kios_robot.data_types import MiosInterfaceResponse, MiosTaskResult
from kios_robot.data_types import MiosSkill, MiosCall
from kios_robot.data_types import TaskScene


class RobotCommand:
    robot_address: str = None
    robot_port: int = None
    shared_data: Any = None

    task_scene: TaskScene = None  # currently not used

    task_list: List[MiosSkill or MiosCall] = []

    def __init__(
        self,
        robot_address: str,
        robot_port: int,
        shared_data: Any,
        task_scene: TaskScene,
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
            pass

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

    def execute_task_list_sync(self) -> bool:
        for task_item in self.task_list:
            if isinstance(task_item, MiosSkill):  # * use general task
                mios_task = Task(self.robot_address)
                mios_task.add_skill(
                    task_item.skill_name,
                    task_item.skill_type,
                    task_item.skill_parameters,
                )
                start_result = mios_task.start()
                mios_start_response = MiosInterfaceResponse.from_json(
                    start_result["result"]
                )
                # print("Result: " + str(result))
                print("\033[92mMios replied: ")
                print(mios_start_response)
                print("\033[0m")

                # * check if the task is started successfully
                if mios_start_response.has_finished == False:
                    raise Exception("task failed to start")

                result = mios_task.wait()
                mios_response = MiosInterfaceResponse.from_json(result["result"])
                # print("Result: " + str(result))
                print("\033[92mMios replied: ")
                print(mios_response)
                print("\033[0m")  # Reset color to default

                # * if the task is not finished successfully, return a false
                if mios_response.has_finished == False:
                    return False
                if mios_response.task_result.has_succeeded == False:
                    return False

            if isinstance(task_item, MiosCall):  # * use general call
                result = call_method(
                    self.robot_address,
                    self.robot_port,
                    task_item.method_name,
                    task_item.method_payload,
                )
                print(result)
                mios_response = MiosInterfaceResponse.from_json(result["result"])
                # print("Result: " + str(result))
                print("\033[92mMios replied: ")
                print(mios_response)
                print("\033[0m")  # Reset color to default
                if mios_response.has_finished == False:
                    return False

        return True

    def add_mios_task(self, mios_task: MiosSkill or MiosCall):
        self.task_list.append(mios_task)
