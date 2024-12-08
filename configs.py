from dotenv import load_dotenv
import os

load_dotenv()


is_simulation = False
robot_id = int(os.getenv("ROBOT_ID"))

raspi_ip = os.getenv("RASPI_IP")
raspi_port = int(os.getenv("RASPI_PORT"))
zinq_ip = os.getenv("ZINQ_IP")
zinq_port = int(os.getenv("ZINQ_PORT"))
orin_ip = os.getenv("ORIN_IP")
orin_port = int(os.getenv("ORIN_PORT"))
server_ip = os.getenv("SERVER_IP")
server_port = int(os.getenv("SERVER_PORT"))

bypass_integrity = True

state = "initial"
