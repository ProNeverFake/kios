from multiprocessing import Process, Manager
from time import sleep


class Task:
    def __init__(self, shared_data):
        self.shared_data = shared_data

    def update(self, value):
        self.shared_data["data"] = value

    def get_data(self):
        return self.shared_data["data"]


def worker(shared_task):
    for value in range(5, 21, 5):
        sleep(5)
        shared_task.update(value)
        print(f"Subprocess: {shared_task.get_data()}")


if __name__ == "__main__":
    with Manager() as manager:
        # Create a shared dictionary
        shared_data = manager.dict({"data": 0})

        # Create a shared version of Task
        shared_task = Task(shared_data)

        # Create and start the subprocess
        p = Process(target=worker, args=(shared_task,))
        p.start()

        while shared_task.get_data() < 20:
            print(f"Main process: {shared_task.get_data()}")
            sleep(1)

        p.join()

        # Final value in the main process
        print(f"Main process final value: {shared_task.get_data()}")
