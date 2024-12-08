from dotenv import load_dotenv
import os

load_dotenv()


is_simulation = False
robot_id = int(os.getenv("ROBOT_ID"))

raspi_ip = os.getenv("RASPI_IP")
raspi_port_server = int(os.getenv("RASPI_PORT_SERVER"))
raspi_port_client = int(os.getenv("RASPI_PORT_CLIENT"))

zinq_ip = os.getenv("ZINQ_IP")
zinq_port_secure = int(os.getenv("ZINQ_PORT_SECURE"))
zinq_port_insecure = int(os.getenv("ZINQ_PORT_INSECURE"))

orin_ip = os.getenv("ORIN_IP")
orin_port = int(os.getenv("ORIN_PORT"))

dashboard_ip = os.getenv("DASHBOARD_IP")
dashboard_port = int(os.getenv("DASHBOARD_PORT"))

bypass_integrity = True

state = "initial"
