import socket
import threading
import configs
from edabot_protocol import send_packet, receive_packet


class ExternalCom:
    def __init__(self):
        # Initialize Raspi as Server for Dashboard
        self.raspi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.raspi_socket.bind((configs.raspi_ip, configs.raspi_port_server))
        self.raspi_socket.listen(5)

        if not configs.bypass_integrity:
            # Initialize Raspi as Client for Zinq
            self.zinq_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.zinq_socket.connect((configs.zinq_ip, configs.zinq_port_secure))

        self.dashboard_client = None

        # Start threads for receiving and sending messages
        self.start()

    def start(self):
        # Initialize threads, must be 4 active threads, see internal_communication.py and simulate_zinq.py for inspiration
        threading.Thread(target=self.receive_messages, args=(self.raspi_socket,)).start()
        threading.Thread(target=self.send_messages, args=(self.raspi_socket,)).start()
        threading.Thread(target=self.accept_dashboard_client).start()
        if not configs.bypass_integrity:
            threading.Thread(target=self.receive_messages, args=(self.zinq_socket,)).start()
            threading.Thread(target=self.send_messages, args=(self.zinq_socket,)).start()
            threading.Thread(target=self.receive_messages_from_dashboard).start()

    def accept_dashboard_client(self):
        client_socket, addr = self.raspi_socket.accept()
        self.dashboard_client = client_socket
        print(f"Dashboard client connected from {addr}")

    def receive_messages(self, sock):
        # Create handler for receiving message
        while True:
            try:
                message = receive_packet(sock)
                if message:
                    print(f"Received: {message}")
                    self.forward_to_dashboard_client(message)
            except Exception as e:
                print(f"Error receiving message: {e}")
                break

    def send_messages(self, sock):
        # Create handler for sending message
        while True:
            try:
                message = input("Enter message to send: ")
                send_packet(sock, message)
            except Exception as e:
                print(f"Error sending message: {e}")
                break

    def forward_to_dashboard_client(self, message):
        if self.dashboard_client:
            try:
                send_packet(self.dashboard_client, message)
            except Exception as e:
                print(f"Error forwarding message to dashboard client: {e}")

    def receive_messages_from_dashboard(self):
        while True:
            try:
                message = receive_packet(self.dashboard_client)
                if message:
                    print(f"Received from Dashboard: {message}")
                    self.forward_to_zinq_secure(message)
            except Exception as e:
                print(f"Error receiving message from Dashboard: {e}")
                break

    def forward_to_zinq_secure(self, message):
        try:
            send_packet(self.zinq_socket, message)
        except Exception as e:
            print(f"Error forwarding message to Zinq (secure): {e}")


if __name__ == "__main__":
    external_com = ExternalCom()
