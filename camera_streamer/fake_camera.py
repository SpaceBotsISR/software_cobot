import socket
import numpy as np

# Image dimensions
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 540
FRAME_SIZE = DISPLAY_WIDTH * DISPLAY_HEIGHT * 3

def generate_random_image_data():
    # Generate random image data for simulation
    return np.random.randint(0, 256, FRAME_SIZE, dtype=np.uint8).tobytes()

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("127.0.0.1", 8080))
    server_socket.listen(1)
    print("Waiting for connection...")

    client_socket, addr = server_socket.accept()
    print(f"Accepted connection from {addr}")

    try:
        while True:
            # Generate and send random image data
            image_data = generate_random_image_data()
            bytes_sent = 0
            while bytes_sent < FRAME_SIZE:
                sent = client_socket.send(image_data[bytes_sent:])
                if sent == 0:
                    raise RuntimeError("Socket connection broken")
                bytes_sent += sent
    except KeyboardInterrupt:
        print("\nStopping image stream...")
    finally:
        client_socket.close()
        server_socket.close()

if __name__ == "__main__":
    main()
