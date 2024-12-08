import socket
import time
import threading

def udp_broadcast_sender():
    broadcast_address = ("255.255.255.255", 5005)
    message = b"Hello! dari wicak"

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            print(f"Sending: {message.decode()}")
            sock.sendto(message, broadcast_address)
            time.sleep(1)

def udp_broadcast_receiver():
    listen_address = ("", 5005)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(listen_address)

        sock.settimeout(10.0)

        print("Listening for broadcast messages...")

        while True:
            try:
                pesan, address = sock.recvfrom(1024)
                # if address[0] == "192.168.2.109":
                #     continue
                print(f"Received from {address}: {pesan.decode()}")
            except socket.timeout:
                print("No messages received. Timeout occurred.")
                break

if __name__ == "__main__":
    sender = threading.Thread(target=udp_broadcast_sender)
    receiver = threading.Thread(target=udp_broadcast_receiver)

    sender.start()
    receiver.start()