import zmq

class ZMQManager:
    def __init__(self, host = '127.0.0.1', port = 8004):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://{host}:{port}")

    def send(self, msg):
        self.socket.send_string(msg)

    def senf_json(self, msg: dict):
        self.socket.send_json(msg)

    def receive(self):
        return self.socket.recv_string()

    def close(self):
        self.socket.close()
        self.context.term()