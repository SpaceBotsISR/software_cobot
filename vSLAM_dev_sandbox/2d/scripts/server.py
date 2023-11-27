import socket


class Server:
    def __init__(
        self,
        server_ip: str = "127.0.0.1",
        client_ip: str = "127.0.0.1",
        server_port: int = 8008,
        client_port: int = 8009,
    ) -> None:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (server_ip, server_port)
        self.client_address = (client_ip, client_port)
        print(
            f"Server is running on {self.server_address[0]}:{self.server_address[1]}\n"
        )

    def send_message(self, msg: str) -> None:
        self.server_socket.sendto(msg.encode("utf-8"), self.client_address)

    def receive_message(self) -> tuple[float, float, float]:
        data, _ = self.server_socket.recvfrom(1024)
        received_message = data.decode("utf-8")

        print(f"Received message: {received_message.split(' ')}")
        x, y, theta, _ = received_message.split(" ")
        return float(x), float(y), float(theta)
