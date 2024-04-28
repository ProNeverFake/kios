#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

from kios_utils.task import *

import atexit
import multiprocessing
import multiprocessing.connection
import time

from kios_robot.robot_command import RobotCommand
from kios_utils.bblab_utils import setup_logger

logger = setup_logger(__name__, logging.DEBUG)


def mios_monitor(
    task: Task, pipe_connection: multiprocessing.connection.Connection
) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the mios_monitor process
    """
    try:
        task.interrupt()
        task.start()
        # hanlde startup failure
        # ! check the response here
        logger.info(str(task.task_start_response))

        task.shared_data["task_start_response"] = task.task_start_response

        if bool(task.task_start_response["result"]["result"]) == False:
            pipe_connection.send([False])
            return

        _ = task.wait()

        if bool(task.task_wait_response["result"]["result"]) == True:
            pipe_connection.send([True])
        else:
            pipe_connection.send([False])
    except KeyboardInterrupt:
        pass


def fake_monitor(
    task: Task, pipe_connection: multiprocessing.connection.Connection
) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the mios_monitor process
    """
    try:
        # * skip
        # task.interrupt()
        # task.start()
        # hanlde startup failure
        # ! check the response here

        time.sleep(2)
        # * fake a start response
        task.shared_data["task_start_response"] = {"result": {"result": True}}

        # * skip the start
        # if bool(task.task_start_response["result"]["result"]) == False:
        #     pipe_connection.send([False])
        #     return
        # * skip the wait
        # _ = task.wait()

        # * sleep for 5 seconds
        time.sleep(5)

        # * skip the wait response
        # if bool(task.task_wait_response["result"]["result"]) == True:
        #     pipe_connection.send([True])
        # else:
        #     pipe_connection.send([False])

        # * send fake response
        logger.info("send fake response...")
        pipe_connection.send([True])

    except KeyboardInterrupt:
        pass


def robot_command_monitor(
    robot_command: RobotCommand, pipe_connection: multiprocessing.connection.Connection
) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the mios_monitor process
    """
    # ! BBBUG 02042024 multiprocessing virtual memory.
    """
    The inconsistency of the scene data between the robot_command and the btw is due to the feature of the multiprocessing module.

    It starts a new process and the data is copied to the new process in a new allocated memory. The data is not naturely shared between the processes.

    The data is shared between the processes by using the multiprocessing module's shared memory feature.

    By printing the address using hex id, the virtual memory address of the scene object is printed. However, the addresses are the same address in the main process and the new process. This is because id() returns the virtual memory address of the object, while the physical memory address is hidden and managed by python. 
    
    The virtual memory address is the same in the main process and the new process, but the physical memory address is different.

    """
    try:
        robot_command.interrupt()
        response = robot_command.execute_task_list_sync()

        if response == False:
            pipe_connection.send([False])
            return

        if response == True:
            pipe_connection.send([True])
            return

    except KeyboardInterrupt:
        pass


def fake_robot_command_monitor(
    robot_command: RobotCommand, pipe_connection: multiprocessing.connection.Connection
) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the mios_monitor process
    """
    try:
        time.sleep(2)
        # * fake a start response

        # * skip the start

        # * skip the wait

        # * sleep for 5 seconds
        time.sleep(5)

        # * skip the wait response

        # * send fake response
        logger.info("send fake response...")
        pipe_connection.send([True])

    except KeyboardInterrupt:
        pass


def robot_command_monitor_fix_try(
    robot_command: RobotCommand, pipe_connection: multiprocessing.connection.Connection
) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the mios_monitor process
    """

    try:
        robot_command.interrupt()
        response = robot_command.execute_task_list_sync()

        if response == False:
            pipe_connection.send([False])
            return

        if response == True:
            pipe_connection.send([True])
            return

    except KeyboardInterrupt:
        pass
