import os
from enum import Enum
from dotenv import load_dotenv
from rcl_interfaces.msg import Log

load_dotenv()


class LogLevels(Enum):
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50


# TODO: make State Handler process to change state between multiple processes
class States(Enum):
    MALFUNCTION = -1
    INIT = 0
    READY = 1
    BUSY = 2


is_simulation = False

log_level = LogLevels.WARN
state = States.INIT
robot_id = int(os.getenv("ROBOT_ID"))

raspi_ip_eksternal = os.getenv("RASPI_IP_EKSTERNAL")
raspi_ip_internal = os.getenv("RASPI_IP_INTERNAL")
raspi_port_internal = int(os.getenv("RASPI_PORT_INTERNAL"))
raspi_port_eksternal = int(os.getenv("RASPI_PORT_EKSTERNAL"))

zinq_ip = os.getenv("ZINQ_IP")
zinq_port_secure = int(os.getenv("ZINQ_PORT_SECURE"))
zinq_port_insecure = int(os.getenv("ZINQ_PORT_INSECURE"))

orin_ip = os.getenv("ORIN_IP")
orin_port = int(os.getenv("ORIN_PORT"))

dashboard_ip = os.getenv("DASHBOARD_IP")
dashboard_port = int(os.getenv("DASHBOARD_PORT"))

bypass_integrity = True
