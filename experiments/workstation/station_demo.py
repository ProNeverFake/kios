import queue
import threading
import socket
from runtime_script import n2_task as nx
from runtime_script import sp2 as spx

# Create a queue to contain tasks
task_queue = queue.Queue()

this_ip = "10.157.174.230"


# Function to execute a task
def execute_task(task_function):
    task_function()


# Function to listen for UDP messages
def listen_udp():
    # Create a UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((this_ip, 12321))

    while True:
        # Receive UDP message
        data, addr = udp_socket.recvfrom(1024)
        message = data.decode("utf-8")

        # Add a special task to the queue
        task_queue.put(spx)


# Start a thread to listen for UDP messages
udp_thread = threading.Thread(target=listen_udp)
udp_thread.start()

while True:

    if task_queue.empty():
        task_queue.put(nx)
    # Pop a task from the queue
    task = task_queue.get()

    # Execute the task
    execute_task(task)

#! AIS
