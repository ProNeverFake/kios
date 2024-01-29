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
        print(str(task.task_start_response))

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
        # print(str(task.task_start_response))

        time.sleep(2)
        # * fake a start response
        # print("start fake monitor")
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
        print("send fake response")
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
        print("send fake response")
        pipe_connection.send([True])

    except KeyboardInterrupt:
        pass
