import socket
import threading
from edabot_protocol import receive_packet
import configs

clients = {}
clients_lock = threading.Lock()


def handle_client(client_socket, client_address):
    """Handle communication with a single client."""
    print(f"New connection from {client_address}")
    try:
        while True:
            message = receive_packet(client_socket)
            if message:
                print(f"Received from {client_address}: {message}")
            else:
                break
    except Exception as e:
        print(f"Error with client {client_address}: {e}")
    finally:
        with clients_lock:
            # Remove client from the active clients list
            for name, sock in list(clients.items()):
                if sock == client_socket:
                    del clients[name]
                    break
        client_socket.close()
        print(f"Connection with {client_address} closed.")


def server():
    """Start the server and handle incoming connections."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((configs.zinq_ip, configs.zinq_port))
    server_socket.listen(2)
    print(f"Server is listening on {configs.zinq_ip}:{configs.zinq_port}...")

    while True:
        try:
            client_socket, client_address = server_socket.accept()
            # Add the client to the clients dictionary
            with clients_lock:
                clients[str(client_address)] = client_socket
            client_thread = threading.Thread(
                target=handle_client, args=(client_socket, client_address), daemon=True
            )
            client_thread.start()
        except KeyboardInterrupt:
            print("Server shutting down.")
            break
        except Exception as e:
            print(f"Server error: {e}")

    server_socket.close()


if __name__ == "__main__":
    server()