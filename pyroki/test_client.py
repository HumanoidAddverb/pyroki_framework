import socket
import json

class PolicyServer:
    def __init__(self, host="0.0.0.0", port=8080, buffer_size=4096):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print(f"PolicyServer listening on {self.host}:{self.port}")

    def start(self):
        conn, addr = self.sock.accept()
        print(f"Client connected: {addr}")
        with conn:
            while True:
                data = conn.recv(self.buffer_size)
                if not data:
                    print("Client disconnected")
                    break

                # decode observation from client
                observation = data.decode("utf-8").strip("\x00")
                print(f"Received raw: {observation}")

                try:
                    obs_json = json.loads(observation)
                    print(f"Decoded observation: {type(obs_json['state']['jpos'])}")
                except json.JSONDecodeError:
                    print("Invalid JSON received")
                    continue

                # Dummy policy: echo action based on observation
                action = {
                    "action": [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]],
                    "time_stamp": [0.0, 0.0]
                }

                response = json.dumps(action)
                conn.sendall(response.encode("utf-8"))  # no \0 needed if C++ uses assign(buffer, ret)
                print(f"Sent action: {response}")


if __name__ == "__main__":
    server = PolicyServer(host="0.0.0.0", port=8080)
    server.start()


def recieve_request(self):
        """Receive current state and target pose from client."""
        if not self._initialised:
            raise RuntimeError("Call initialise() first")

        data = self.conn.recv(4096)
        if not data:
            print("Client disconnected, waiting again...")
            self.listen()
            data = self.conn.recv(4096)

        try:
            message = json.loads(data.decode("utf-8").strip("\x00"))
        except json.JSONDecodeError:
            print("Invalid JSON received")
            return False

        # First dof entries are joint state
        self.params.state = message['state']['jpos']

        assert len(self.params.state) == self.params.dof, f"State must have {self.params.dof} values, got {len(self.params.state)}"

        # Remaining entries are pose
        pose = message['state']['target_pose']
        assert len(pose) == 7, f"Pose must have 7 values, got {len(pose)}"

        self.params.target.position = pose[:3]
        self.params.target.orientation = pose[3:]

        print(f"Received state: {self.params.state}")
        print(f"Received target pose: {pose}")
        return True

def send_response(self):
        """Send control input to client."""
        if not self._initialised:
            raise RuntimeError("Call initialise() first")

        if not self.params.control:
            # Example dummy control
            self.params.control = {
                "action": [[0.0] * 7, [0.0] * 7],
                "time_stamp": [0.0, 0.0]
            }

        control_json = {
            "action": self.params.control["control"],
            "time_stamp": self.params.control["dt"]
        }

        msg = json.dumps(control_json).encode("utf-8")

        self.conn.sendall(msg)  # raw JSON
        print(f"Sent response to client: {self.params.control}")