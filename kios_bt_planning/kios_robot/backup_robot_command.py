from kios_utils.task import *
import time
from tabulate import tabulate
from pprint import pprint

'''
just for backup. the implementation is dirty. Can remove this I guess.
'''

from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import MiosInterfaceResponse, MiosTaskResult
from kios_robot.data_types import MiosSkill, MiosCall, KiosCall
from kios_robot.data_types import TaskScene

from kios_utils.bblab_utils import bb_result_test


class RetryException(Exception):
    pass


class FailureException(Exception):
    pass


class RobotCommand:
    robot_address: str = None
    robot_port: int = None
    shared_data = None

    robot_interface: RobotInterface = None

    task_scene: TaskScene = None

    task_list: list[MiosSkill | MiosCall | KiosCall] = []

    def __init__(
        self,
        robot_address: str,
        robot_port: int,
        shared_data,
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
        self.task_list = []

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

    # TODO bad practice. use several methods instead.
    def execute_task_list_sync(self) -> bool:
        self.show_tasks()
        i = 0
        for i in range(len(self.task_list)):
            task_item = self.task_list[i]
            # * MiosSkill
            if isinstance(task_item, MiosSkill):
                mios_task = Task(self.robot_address)
                mios_task.add_skill(
                    task_item.skill_name,
                    task_item.skill_type,
                    task_item.skill_parameters,
                )
                start_result = mios_task.start()

                # handle the start response
                if start_result is None:
                    # failed to start the task.
                    if task_item.retry == True:
                        continue
                    else:
                        return False
                mios_start_response = MiosInterfaceResponse.from_json(
                    start_result["result"]
                )
                print("\033[92mMiosSkill started: ")
                pprint(mios_start_response)
                print("\033[0m")
                # handle the failure in starting the task
                if mios_start_response.has_finished == False:
                    if task_item.retry == True:
                        continue
                    else:
                        return False

                # wait for the task to finish
                result = mios_task.wait()
                if result is None:
                    if task_item.retry == True:
                        continue
                    else:
                        return False
                mios_response = MiosInterfaceResponse.from_json(result["result"])
                print("\033[92mMios replied: ")
                pprint(mios_response)
                print("\033[0m")

                # if the task is not finished successfully, return a false
                if mios_response.has_finished == False:
                    if task_item.retry == True:
                        continue
                    else:
                        return False
                if mios_response.task_result.has_succeeded == False:
                    if task_item.retry == True:
                        continue
                    else:
                        return False

            elif isinstance(task_item, MiosCall):  # * use general call
                print(f"\033[92mStart the call: {task_item.method_name}\033[0m")
                result = call_method(
                    self.robot_address,
                    self.robot_port,
                    task_item.method_name,
                    task_item.method_payload,
                )
                if result is None:
                    if task_item.retry == True:
                        continue
                    else:
                        return False
                mios_response = MiosInterfaceResponse.from_json(result["result"])
                print("\033[92mMios replied: ")
                print(mios_response)
                print("\033[0m")
                if mios_response.has_finished == False:
                    if task_item.retry == True:
                        continue
                    else:
                        return False

            elif isinstance(task_item, KiosCall):
                result_bool = task_item.method(*task_item.args)
                if result_bool != True:
                    return False

            else:
                raise Exception("Unknown task type: {}".format(task_item))

            # go to next task
            i += 1

        print("\033[94mRobot command has successfully finished\033[0m")  # Print in blue
        return True

    def add_task(self, task: MiosSkill | MiosCall | KiosCall):
        self.task_list.append(task)

    def add_tasks(self, tasks: list[MiosSkill | MiosCall | KiosCall]):
        self.task_list.extend(tasks)

    def prepend_task(self, task: MiosSkill | MiosCall | KiosCall):
        self.task_list.insert(0, task)

    def prepend_tasks(self, tasks: list[MiosSkill | MiosCall | KiosCall]):
        self.task_list = tasks + self.task_list

    def clear_tasks(self):
        self.task_list = []

    def show_tasks(self):
        task_table = []
        for task in self.task_list:
            task_type = type(task).__name__  # mios_skill, mios_call, kios_call
            task_name = str(task)
            task_table.append([task_type, task_name])

        print(tabulate(task_table, headers=["Task Type", "Task Name"], tablefmt="grid"))
