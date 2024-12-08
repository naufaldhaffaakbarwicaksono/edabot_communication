import socket
import threading
import time
from edabot_protocol import receive_packet, send_packet


def start_server(host="127.0.0.1", port=65432):
    with socket.create_server((host, port)) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        conn, _ = server_socket.accept()

        message = receive_packet(conn, simple_protocol=False)
        if message:
            print("Received from client:", message)


def start_client(host="127.0.0.1", port=65432):
    with socket.create_connection((host, port)) as client_socket:
        message = input("Send to client: ")
        if not send_packet(client_socket, message, simple_protocol=False):
            print("Send packet failed")


if __name__ == "__main__":
    server = threading.Thread(target=start_server)
    server.start()

    time.sleep(1)

    client = threading.Thread(target=start_client)
    client.start()

    server.join()
    client.join()
