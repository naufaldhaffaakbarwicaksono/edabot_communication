import socket
import threading
import time
import json  # Add this line

# Read IP and port from robot_ip.txt
with open('robot_ip.txt', 'r') as f:
    SERVER_IP = f.readline().strip()
    SERVER_PORT = int(f.readline().strip())

def send_broadcast_message(message, port=SERVER_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Add reuse address option
    broadcast_address = ('192.168.2.255', port)
    sock.sendto(message.encode(), broadcast_address)
    sock.close()

def receive_broadcast_message(port=SERVER_PORT):  # Modify this line
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', port))
    local_ip = SERVER_IP  # Use the IP from the file
    while True:
        data, addr = sock.recvfrom(1024)
        if addr[0] != local_ip:  # Prevent loopback
            message = data.decode()  # Decode the message
            print(f"Received message: {message} from IP: {addr[0]}")

def start_receiving():  # Modify this line
    recv_thread = threading.Thread(target=receive_broadcast_message, args=(SERVER_PORT,))  # Modify this line
    recv_thread.daemon = True
    recv_thread.start()

def send_continuously(interval=2):
    while True:
        position_data = {
            "command": "other",
            "robot": "pose_robot1", # Change this to the robot label
            "pose": "example_pose"  # Replace with actual pose data
        }
        data_to_broadcast = json.dumps(position_data)
        send_broadcast_message(data_to_broadcast)
        time.sleep(interval)

if __name__ == "__main__":
    start_receiving()  # Modify this line
    send_continuously()  # Modify this line
