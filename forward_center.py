import socket
from multiprocessing import SimpleQueue
import time

def connect_to_forward_tcp(sock):
    while True:
        try:
            conn, addr = sock.accept()
            return conn, addr
        except socket.error:
            print("Waiting for client to connect...")
            time.sleep(5)  # Wait before retrying
            continue

def run(to_cc, from_cc):
    # Read server IP and port from file
    with open('tcp_ip.txt', 'r') as f:
        SERVER_IP = f.readline().strip()
        SERVER_PORT = int(f.readline().strip())
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((SERVER_IP, SERVER_PORT))
    sock.listen(1)
    conn, addr = connect_to_forward_tcp(sock)

    # terima data dari forward_tcp.py dan lalukan forward ke client
    def forward_to_client():
        nonlocal conn
        while True:
            try:
                if not to_cc.empty():
                    data = to_cc.get()
                    conn.sendall(data)
            except socket.error:
                print("Client disconnected, waiting for new client...")
                conn, addr = connect_to_forward_tcp(sock)

    #terima data_command dari command control dan teruskan ke forward_tcp.py
    def receive_from_client():
        nonlocal conn
        while True:
            try:
                data = conn.recv(4096)
                if data:
                    from_cc.put(data)
            except socket.error:
                print("Client disconnected, waiting for new client...")
                conn, addr = connect_to_forward_tcp(sock)

    import threading
    threading.Thread(target=forward_to_client).start()
    threading.Thread(target=receive_from_client).start()

if __name__ == "__main__":
    to_cc = SimpleQueue()
    from_cc = SimpleQueue()
    run(to_cc, from_cc)