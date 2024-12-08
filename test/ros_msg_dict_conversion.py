from sensor_msgs.msg import Imu
from utils import ros_msg_to_dict, dict_to_ros_msg


ros_msg = Imu()
print("Original ros msg:", ros_msg)

dict_msg = ros_msg_to_dict(ros_msg)
print("Convert to dict:", dict_msg)

ros_msg_back = dict_to_ros_msg(Imu, dict_msg)
print("Convert back to ros msg:", ros_msg_back)

if ros_msg_back == ros_msg:
    print("Convert process success")
else:
    print("Convert process failed")
