import socket
from multiprocessing import SimpleQueue
import aioprocessing as ap
import asyncio
import threading
import time

to_cc = SimpleQueue()
from_cc = SimpleQueue()

SERVER_IP = '192.168.1.2'
SERVER_PORT = 48101

def connect_to_server():
    while True:
        try:
            conn = socket.create_connection((SERVER_IP, SERVER_PORT))
            return conn
        except socket.error:
            print("Waiting for server to be active...")
            time.sleep(5)  
            continue

def receive_from_server():
    conn = connect_to_server()
    while True:
        try:
            data_from_controller = conn.recv(4096)
            if data_from_controller:
                to_cc.put(data_from_controller)
        except socket.error:
            print("Server disconnected, reconnecting...")
            conn = connect_to_server()

# Function to receive data from from_cc and forward to bismillah_server.py
def send_to_server():
    conn = connect_to_server()
    while True:
        try:
            if not from_cc.empty():
                data_from_cc = from_cc.get()
                conn.sendall(data_from_cc)
        except socket.error:
            print("Server disconnected, reconnecting...")
            conn = connect_to_server()

# Function to start forward_center.py
def start_forward_center(to_cc, from_cc):
    import forward_center
    forward_center.run(to_cc, from_cc)

async def main():
    process_center = ap.AioProcess(target=start_forward_center, args=(to_cc, from_cc))
    process_center.start()

    receive_thread = threading.Thread(target=receive_from_server)
    send_thread = threading.Thread(target=send_to_server)
    receive_thread.start()
    send_thread.start()

    await process_center.coro_join()

    receive_thread.join()
    send_thread.join()

if __name__ == "__main__":
    asyncio.run(main())