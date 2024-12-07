import socket
import threading
from time import sleep

clients = {}


def handle_client(client_socket):
    try:
        # Receive the client's name
        client_name = client_socket.recv(1024).decode("utf-8")
        clients[client_name] = client_socket
        print(f"{client_name} connected.")

        while True:
            header = client_socket.recv(1).decode()
            if header != 0xFE:
                return None

            length = int(client_socket.recv(10).decode())
            print(length)

            # data_length = int(header.strip())
            # print(data_length)

            print(f"Message from {client_name}: {length}")

    except ConnectionResetError:
        print(f"{client_name} disconnected.")
    finally:
        # Clean up when a client disconnects
        del clients[client_name]
        client_socket.close()


def broadcast_message(message, sender_name):
    for name, client_socket in clients.items():
        if name != sender_name:  # Don't send the message to the sender
            try:
                client_socket.send(message.encode("utf-8"))
            except BrokenPipeError:
                print(f"Could not send to {name}. Client disconnected.")


def send_to_client(client_name, message):
    while True:
        if client_name in clients:
            try:
                clients[client_name].send(message.encode("utf-8"))
                print(f"Sent to {client_name}: {message}")
            except BrokenPipeError:
                print(f"Failed to send message to {client_name}. Client disconnected.")
        else:
            print(f"Client {client_name} not found.")
        sleep(1)


def server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("0.0.0.0", 9999))
    server_socket.listen(5)
    print("Server is listening on port 9999...")

    # threading.Thread(
    #     target=send_to_client, args=("orin", "Hello from server")
    # ).start()  # Start the server console for admin

    while True:
        client_socket, client_address = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()


if __name__ == "__main__":
    server()
