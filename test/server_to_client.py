import socket
import threading
import time
from edabot_protocol import receive_packet, send_packet


def start_server(host="127.0.0.1", port=65432):
    with socket.create_server((host, port)) as server_socket:
        conn, _ = server_socket.accept()

        message = input("Send to client: ")
        if not send_packet(conn, message, simple_protocol=False):
            print("Failed to send packet to client")


def start_client(host="127.0.0.1", port=65432):
    with socket.create_connection((host, port)) as client_socket:
        message = receive_packet(client_socket, simple_protocol=False)
        print("Client received:", message or "No message received from server")


if __name__ == "__main__":
    server = threading.Thread(target=start_server)
    server.start()

    time.sleep(1)

    client = threading.Thread(target=start_client)
    client.start()

    server.join()
    client.join()
