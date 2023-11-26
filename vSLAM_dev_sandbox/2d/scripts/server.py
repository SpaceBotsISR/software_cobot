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

    def send_message(self, message: str) -> None:
        self.server_socket.sendto(message.encode("utf-8"), self.client_address)
