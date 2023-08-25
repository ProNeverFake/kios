import socket
import threading
from collections import deque


class UDPReceiver:
    def __init__(self, host="localhost", port=12346, buffer_size=1024):
        self.buffer_size = buffer_size
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.queue = deque()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._receive_msg)

    def start(self):
        self.running = True
        self.thread.start()

    def _receive_msg(self):
        while self.running:
            data, _ = self.socket.recvfrom(self.buffer_size)
            with self.lock:
                self.queue.append(data)

    def get_last_message(self):
        with self.lock:
            last_msg = self.queue.pop() if self.queue else None
            self.queue.clear()  # Clear all messages in the queue
            return last_msg

    def stop(self):
        self.running = False
        self.socket.close()
        self.thread.join()


if __name__ == "__main__":
    udp_receiver = UDPReceiver()

    # Example: Using the udp_receiver in another thread to fetch the last message
    def fetch_message():
        while True:
            msg = udp_receiver.get_last_message()
            if msg:
                print("Last message received:", msg.decode())

    fetch_thread = threading.Thread(target=fetch_message)
    fetch_thread.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        udp_receiver.stop()
        fetch_thread.join()
