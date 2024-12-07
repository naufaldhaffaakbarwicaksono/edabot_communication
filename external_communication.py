import socket
import configs
import threading
import time


def udp_broadcast_sender():
    broadcast_address = (configs.broadcast_ip, configs.broadcast_port)
    message = b"Hello, WSL"

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            print(f"Sending: {message.decode()}")
            sock.sendto(message, broadcast_address)
            time.sleep(1)


def udp_broadcast_receiver():
    listen_address = ("", configs.broadcast_port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(listen_address)

        print("Listening for broadcast messages...")

        while True:
            try:
                message, address = sock.recvfrom(1024)
                if address[0] == "172.25.59.215":
                    continue
                print(f"Received from {address}: {message.decode()}")
            except socket.timeout:
                print("No messages received. Timeout occurred.")
                break


if __name__ == "__main__":
    sender = threading.Thread(target=udp_broadcast_sender)
    receiver = threading.Thread(target=udp_broadcast_receiver)

    sender.start()
    receiver.start()
