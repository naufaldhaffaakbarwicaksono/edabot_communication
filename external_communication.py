import socket
import json
import threading
import configs
from edabot_protocol import send_packet, receive_packet


class ExternalCom:
    def __init__(self):
        # TODO: Initialize Raspi as Server for Dashboard
        self.raspi_socket = ...

        if not configs.bypass_integrity:
            # TODO: Initialize Raspi as Client for Zinq
            self.zinq_socket = ...
            pass

        # Start threads for receiving and sending messages
        self.start()

    def start(self):
        # TODO: Initialize threads, must be 4 active threads, see internal_communication.py and simulate_zinq.py for inspiration
        pass

    def receive_messages(self, sock):
        # TODO: Create handler for receiving message
        pass

    def send_messages(self, sock):
        # TODO: Create handler for sending message
        pass


if __name__ == "__main__":
    external_com = ExternalCom()
