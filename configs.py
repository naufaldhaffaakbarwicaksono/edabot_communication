from dotenv import load_dotenv
import os

load_dotenv()


env = "simulation"  # or production

tcp_ip = os.getenv("TCP_IP", "127.0.0.1")
tcp_port = int(os.getenv("TCP_PORT", 5000))

broadcast_ip = os.getenv("BROADCAST_IP")
broadcast_port = int(os.getenv("BROADCAST_PORT"))

bypass_integrity = True
