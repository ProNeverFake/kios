# #!/usr/bin/env python

# ##############################################################################
# # Imports
# ##############################################################################

# # ! DISCARDED

# from kios_utils.task import *

# import atexit
# import multiprocessing
# import multiprocessing.connection
# import time

# # ! BBDEV: this should be discarded later.
# # * new func: in robot.skill_engine.


# def mios_monitor(
#     task: Task, pipe_connection: multiprocessing.connection.Connection
# ) -> None:
#     """Emulate a (potentially) long running external process.

#     Args:
#         pipe_connection: connection to the mios_monitor process
#     """
#     try:
#         task.interrupt()
#         task.start()
#         # hanlde startup failure
#         # ! check the response here
#         print(str(task.task_start_response))

#         task.shared_data["task_start_response"] = task.task_start_response

#         if bool(task.task_start_response["result"]["result"]) == False:
#             pipe_connection.send([False])
#             return

#         _ = task.wait()

#         if bool(task.task_wait_response["result"]["result"]) == True:
#             pipe_connection.send([True])
#         else:
#             pipe_connection.send([False])
#     except KeyboardInterrupt:
#         pass


# def fake_monitor(
#     task: Task, pipe_connection: multiprocessing.connection.Connection
# ) -> None:
#     """Emulate a (potentially) long running external process.

#     Args:
#         pipe_connection: connection to the mios_monitor process
#     """
#     try:
#         # * skip
#         # task.interrupt()
#         # task.start()
#         # hanlde startup failure
#         # ! check the response here
#         # print(str(task.task_start_response))

#         time.sleep(2)
#         # * fake a start response
#         # print("start fake monitor")
#         task.shared_data["task_start_response"] = {"result": {"result": True}}

#         # * skip the start
#         # if bool(task.task_start_response["result"]["result"]) == False:
#         #     pipe_connection.send([False])
#         #     return
#         # * skip the wait
#         # _ = task.wait()

#         # * sleep for 5 seconds
#         time.sleep(5)

#         # * skip the wait response
#         # if bool(task.task_wait_response["result"]["result"]) == True:
#         #     pipe_connection.send([True])
#         # else:
#         #     pipe_connection.send([False])

#         # * send fake response
#         print("send fake response")
#         pipe_connection.send([True])

#     except KeyboardInterrupt:
#         pass


# def kios_skill_async():
#     raise NotImplementedError
