from socket import socket
import configs
import hashlib
import zlib


HEAD = b"R" + configs.robot_id.to_bytes(1, byteorder="big")
TAIL = b"e^&og\r\n\r\n"
CHECKSUM_SIZE = 2


def calculate_checksum(payload: str):
    return hashlib.sha256(payload).digest()[:CHECKSUM_SIZE]


def send_packet(sock: socket, payload: str, simple_protocol=True):
    try:
        payload = zlib.compress(payload.encode())
        if simple_protocol:
            packet = HEAD + payload + TAIL
        else:
            length = len(payload).to_bytes(3, byteorder="big")
            checksum = calculate_checksum(payload)

            packet = HEAD + length + payload + checksum

        sock.sendall(packet)
        return True
    except Exception as e:
        print(e)
        return False


def receive_packet(sock: socket, simple_protocol=True):
    try:
        head = sock.recv(2)
        if head != HEAD:
            return

        if simple_protocol:
            message = b""
            while True:
                message = message + sock.recv(1)
                if TAIL in message:
                    break
            payload = message[: -len(TAIL)]

        else:
            length = int.from_bytes(sock.recv(3), byteorder="big")
            if length > 0xFFFFFF:
                return

            message = sock.recv(length + CHECKSUM_SIZE)
            payload = message[:-CHECKSUM_SIZE]
            received_checksum = calculate_checksum(payload)
            if received_checksum != calculate_checksum(payload):
                return

        return zlib.decompress(payload).decode()
    except Exception as e:
        print(e)
        return False
