from kios_utils.task import *
from tabulate import tabulate
from pprint import pprint

import logging
import colorlog

# ! I know this block for colorlog is duplicated. you can find the same thing in bt_stw.py.
handler = colorlog.StreamHandler()
handler.setFormatter(
    colorlog.ColoredFormatter(
        "%(log_color)s%(levelname)s:%(name)s:%(message)s",
        log_colors={
            "DEBUG": "cyan",
            "INFO": "green",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "red,bg_white",
        },
    )
)

logger = logging.getLogger()
logger.addHandler(handler)

from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import MiosInterfaceResponse, MiosTaskResult
from kios_robot.data_types import MiosSkill, MiosCall, KiosCall
from kios_robot.data_types import TaskScene

from kios_utils.bblab_utils import bb_result_test


class TrivialException(Exception):
    pass


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

    def execute_task_list_sync(self) -> bool:
        self.show_tasks()
        for task_item in self.task_list:
            if not self.execute_task(task_item):
                return False

        logging.info("All tasks executed successfully.")
        return True

    def execute_task(self, task_item: MiosSkill | MiosCall | KiosCall) -> bool:
        while True:
            try:
                if isinstance(task_item, MiosSkill):
                    self.execute_mios_skill(task_item)
                elif isinstance(task_item, MiosCall):
                    self.execute_mios_call(task_item)
                elif isinstance(task_item, KiosCall):
                    self.execute_kios_call(task_item)
                else:
                    raise Exception("Unknown task type: {}".format(task_item))
                return True
            except TrivialException as e:
                logging.warning(f"Task {task_item} is defined as trivial. Skip...")
                return True
            except RetryException as e:
                logging.warning(f"Task {task_item} failed. Retrying...")
                continue
            except FailureException:
                logging.error(f"Task {task_item} failed. Stop and return failure...")
                return False

    def try_starting_mios_task(self, mios_task: Task):
        start_result = mios_task.start()
        if start_result is None:
            raise RetryException("Failed to start the task")
        return start_result

    def log_response(self, message, response):
        print("this")
        logging.info(message)
        logging.info(response)
        # print("\033[92m{}\033[0m".format(message))
        # pprint(response)
        # print("\033[0m")

    def raise_exception_for_task(self, task_item: MiosSkill | MiosCall | KiosCall):
        if task_item.isTrivial is not None and task_item.isTrivial:
            raise TrivialException
        elif task_item.retry is not None and task_item.retry:
            raise RetryException
        else:
            raise FailureException

    def check_response(self, response: MiosInterfaceResponse | dict, task_item):
        # TODO this is confusing. someone comes to refactor this plz!
        """check response and raise exceptions accordingly

        Args:
            response: can be a mios interface response so result is checker here, or a dict so this method checks if it is None.

        Raises:

        """
        if isinstance(response, MiosInterfaceResponse):
            if not response.has_finished:
                self.raise_exception_for_task(task_item)
            else:
                if (
                    response.task_result != None
                    and not response.task_result.has_succeeded
                ):
                    self.raise_exception_for_task(task_item)
        else:
            if response is None:
                self.raise_exception_for_task(task_item)

    def execute_mios_skill(self, task_item: MiosSkill):
        mios_task = Task(self.robot_address)
        mios_task.add_skill(
            task_item.skill_name,
            task_item.skill_type,
            task_item.skill_parameters,
        )
        start_result = self.try_starting_mios_task(mios_task)

        mios_start_response = MiosInterfaceResponse.from_json(start_result["result"])
        self.log_response("MiosSkill started: ", mios_start_response)

        result = mios_task.wait()
        self.check_response(result, task_item)
        final_response = MiosInterfaceResponse.from_json(result["result"])
        self.log_response("Mios replied: ", final_response)
        self.check_response(final_response, task_item)

    def execute_mios_call(self, task_item: MiosCall):
        result = call_method(
            self.robot_address,
            self.robot_port,
            task_item.method_name,
            task_item.method_payload,
        )
        # block call, must have a response
        self.check_response(result, task_item)
        mios_response = MiosInterfaceResponse.from_json(result["result"])
        self.log_response("Mios replied: ", mios_response)
        self.check_response(mios_response, task_item)

    def execute_kios_call(self, task_item: KiosCall):
        result_bool = task_item.method(*task_item.args)
        if not result_bool:
            self.raise_exception_for_task(task_item)

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
