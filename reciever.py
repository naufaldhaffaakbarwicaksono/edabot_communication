import socket
import json

FOOTER = "e^&og\r\n\r\n"

def start_client(host="127.0.0.1", port=48101):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((host, port))
            buffer = ""
            while True:
                data = client_socket.recv(4096).decode("utf-8")
                if not data:
                    break
                buffer += data
                while FOOTER in buffer:
                    packet, buffer = buffer.split(FOOTER, 1)
                    if packet.startswith("R1"):
                        raw_data = packet[2:]
                        print("Received pose to other robot data:", raw_data)
                        # Process pose to other robot data here
                    elif packet.startswith("S0"):
                        raw_data = packet[2:]
                        try:
                            control_data = json.loads(raw_data)
                            print("Received robot data:", control_data)
                            # Process robot data here
                        except json.JSONDecodeError as e:
                            print(f"Received invalid JSON data: {e}")
                            print(f"Raw data: {raw_data}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    start_client()