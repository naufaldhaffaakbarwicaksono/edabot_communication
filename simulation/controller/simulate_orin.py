import socket
import multiprocessing
from time import sleep


def receive_messages(sock):
    while True:
        try:
            message = sock.recv(1024).decode("utf-8")
            if not message:
                break
            print(f"Server: {message}")
        except ConnectionResetError:
            break
    print("Server disconnected.")
    sock.close()


def send_messages(sock):
    while True:
        try:
            message = "Hello from orin"
            sock.send(message.encode("utf-8"))
        except (ConnectionResetError, BrokenPipeError):
            break
        sleep(1)
    print("Connection closed.")
    sock.close()


def client():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    client_socket.bind(("0.0.0.0", 12341))
    client_socket.connect(("127.0.0.1", 9999))

    # Provide a name for this client
    client_name = "orin"
    client_socket.send(client_name.encode("utf-8"))
    print(f"Connected to server as {client_name}")

    # Create separate processes for receiving and sending messages
    receive_process = multiprocessing.Process(
        target=receive_messages, args=(client_socket,)
    )
    send_process = multiprocessing.Process(target=send_messages, args=(client_socket,))

    receive_process.start()
    send_process.start()

    # Wait for both processes to complete
    receive_process.join()
    send_process.join()


if __name__ == "__main__":
    client()
