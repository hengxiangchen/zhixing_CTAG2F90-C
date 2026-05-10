import serial
import time
import rospy 
from std_msgs.msg import String

def serial_init():
    ser = serial.Serial('/dev/grap', 115200, timeout=1)
    return ser

def read_status(ser, cmd, expected_len):
    """ 发送Modbus指令并读取完整响应 """
    ser.write(bytes.fromhex(cmd))
    time.sleep(0.05)  # 适当等待响应
    response = ser.read(expected_len)  # 读取完整数据包
    
    if len(response) != expected_len:
        rospy.logwarn(f"Response length mismatch: expected {expected_len}, got {len(response)}")
        return None
    
    return response

def parse_modbus_response(data):
    """ 解析Modbus RTU数据包 """
    if data is None or len(data) < 5:
        return "Invalid Data"

    device_id = data[0]
    function_code = data[1]
    data_length = data[2]
    response_data = data[3:-2]  # 去掉 CRC 校验
    crc = data[-2:]  # CRC 校验码

    return f"ID: {device_id}, Func: {function_code}, Data: {response_data.hex()}"

def get_grap_status(ser):
    """ 读取夹爪状态 """
    position = read_status(ser, "01 03 06 09 00 02 14 81", 9)
    torque = read_status(ser, "01 03 06 0C 00 01 44 81", 7)
    alarm = read_status(ser, "01 03 06 12 00 01 24 87", 7)

    return {
        "Position": parse_modbus_response(position),
        "Torque": parse_modbus_response(torque),
        "Alarm": parse_modbus_response(alarm)
    }

def status_publisher():
    global ser
    pub = rospy.Publisher('grap_status', String, queue_size=10)
    rospy.init_node('grap_status_node', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz 发送状态信息
    
    while not rospy.is_shutdown():
        status = get_grap_status(ser)
        status_str = f"Position: {status['Position']}, Torque: {status['Torque']}, Alarm: {status['Alarm']}"
        
        rospy.loginfo(status_str)
        pub.publish(status_str)
        rate.sleep()

if __name__ == "__main__":
    ser = serial_init()
    try:
        status_publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
