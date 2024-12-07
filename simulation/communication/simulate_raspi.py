import socket
import threading


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


def client():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    client_socket.bind(("0.0.0.0", 12342))
    client_socket.connect(("127.0.0.1", 9999))

    # Provide a name for this client
    client_name = "raspi"
    client_socket.send(client_name.encode("utf-8"))
    print(f"Connected to server as {client_name}")

    # Start thread to receive messages
    receive_thread = threading.Thread(target=receive_messages, args=(client_socket,))
    receive_thread.start()

    while True:
        try:
            message = input("You: ")
            client_socket.send(message.encode("utf-8"))
        except (ConnectionResetError, BrokenPipeError):
            break
    print("Connection closed.")
    client_socket.close()


if __name__ == "__main__":
    client()
