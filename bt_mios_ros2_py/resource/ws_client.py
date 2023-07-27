import json
import websockets
import asyncio
import socket
from concurrent.futures import TimeoutError as ConnectionTimeoutError
import websockets.exceptions
import sys
import os


mios_communication_root = os.path.dirname(__file__)
if mios_communication_root not in sys.path:
    sys.path.insert(0, os.path.abspath(mios_communication_root))
    # print("add path mios_communication_root in the file", __file__)

async def send(hostname, port=12000, endpoint="mios/core", request=None, timeout=100, silent=False):
    """sending msg to the wbesoecket server

    Args:
        hostname (_type_): ip of the mios PC
        port (int, optional): mios port Defaults to 12000 (left: 12000, right: 13000)
        endpoint (str, optional): endpoint. Defaults to "mios/core".
        request (_type_, optional): _description_. Defaults to None.
        timeout (int, optional): _description_. Defaults to 100.
        silent (bool, optional): . Defaults to False.

    Returns:
        _type_: reponse or None
    """
    uri = "ws://" + hostname + ":" + str(port) + "/" +endpoint
    try:
        async with websockets.connect(uri, close_timeout=1000) as websocket:
            message = json.dumps(request)
            await websocket.send(message)
            response = await asyncio.wait_for(websocket.recv(), timeout=timeout)
            return json.loads(response)
    except ConnectionRefusedError as e:
        if silent is False:
            print("ConnectionRefusedError: ")
            print(e)
            print("Hostname: " + hostname + ", port: " + str(port) + ", endpoint: " + endpoint)
        return None
    except ConnectionResetError as e:
        if silent is False:
            print("ConnectionResetError: ")
            print(e)
            print("Hostname: " + hostname + ", port: " + str(port) + ", endpoint: " + endpoint)
        return None
    except ConnectionAbortedError as e:
        if silent is False:
            print("ConnectionAbortedError: ")
            print(e)
            print("Hostname: " + hostname + ", port: " + str(port) + ", endpoint: " + endpoint)
        return None
    except websockets.ConnectionClosedError as e:
        if silent is False:
            print("ConnectionClosedError: ")
            print(e)
            print("Hostname: " + hostname + ", port: " + str(port) + ", endpoint: " + endpoint)
        return None
    except ConnectionTimeoutError as e:
        if silent is False:
            print("ConnectionTimeoutError: ")
            print(e)
            print("Hostname: " + hostname + ", port: " + str(port) + ", endpoint: " + endpoint)
        return None
    except websockets.exceptions.InvalidMessage as e:
        if silent is False:
            print("InvalidMessage: ")
            print(e)
            print("Hostname: " + hostname + ", port: " + str(port) + ", endpoint: " + endpoint)
        return None

def call_server(hostname, port, endpoint, request, timeout):
    asyncio.set_event_loop(asyncio.new_event_loop())
    return asyncio.get_event_loop().run_until_complete(send(hostname, request=request, port=port,
                                                            endpoint=endpoint, timeout=timeout))

def call_method(hostname: str, port: int, method, payload=None, endpoint="mios/core", timeout=100, silent=False):
    """sending request to websocket server

    Args:
        hostname (str): ip of the mios PC
        port (int): mios port Defaults to 12000 (left: 12000, right: 13000)
        method (_type_): name of the registed functions in mios
        payload (_type_, optional): payload, which contains the details of the tasks
        endpoint (str, optional): _description_. Defaults to "mios/core".
        timeout (int, optional): _description_. Defaults to 100.
        silent (bool, optional): _description_. Defaults to False.

    Returns:
        _type_: the return of the registed functions
    """
    try:
        request = {
            "method": method,
            "request": payload
        }
        asyncio.set_event_loop(asyncio.new_event_loop())
        return asyncio.get_event_loop().run_until_complete(send(hostname, request=request, port=port,
                                                                endpoint=endpoint, timeout=timeout, silent=silent))
    except socket.gaierror as e:
        print(e)
        print("Hostname: " + hostname + ", port:" + str(port) + ", endpoint: " + endpoint)
        return None

def start_task(hostname: str, task: str, parameters={}, queue=False):
    """start a task in mios

    Args:
        hostname (str): ip of the mios PC
        task (str): task name
        parameters (dict, optional): _description_. Defaults to {}.
        queue (bool, optional): _description_. Defaults to False.

    Returns:
        _type_: _description_
    """
    payload = {
        "task": task,
        "parameters": parameters,
        "queue": queue
    }
    return call_method(hostname, 12000, "start_task", payload)


def stop_task(hostname: str, raise_exception=False, recover=False, empty_queue=False):
    """start the current executed task in mios

    Args:
        hostname (str): ip of the mios PC
        raise_exception (bool, optional): _description_. Defaults to False.
        recover (bool, optional): _description_. Defaults to False.
        empty_queue (bool, optional): _description_. Defaults to False.

    Returns:
        _type_: _description_
    """
    payload = {
        "raise_exception": raise_exception,
        "recover": recover,
        "empty_queue": empty_queue
    }
    return call_method(hostname, 12000, "stop_task", payload)


def wait_for_task(hostname: str, task_uuid: str):
    """blocking the current proocess until the task finishes

    Args:
        hostname (str): ip of the mios PC
        task_uuid (str): uuid of the task to be stopped

    Returns:
        _type_: _description_
    """
    payload = {
        "task_uuid": task_uuid
    }
    return call_method(hostname, 12000, "wait_for_task", payload)


def start_task_and_wait(hostname, task, parameters, queue=False):
    response = start_task(hostname, task, parameters, queue)
    response = wait_for_task(hostname, response["result"]["task_uuid"])
    return response


