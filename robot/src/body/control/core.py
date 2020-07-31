import serial
import rospy
# from config.config import Config


class Control:
    def __init__(self, light_port_name, light_baud, enable_light):
        # self.config = Config
        # port = self.config.light_port_name
        # baud = self.config.light_baud
        port = light_port_name
        baud = light_baud

        if enable_light:
            self.ser = serial.Serial(port, baud, bytesize=serial.EIGHTBITS, timeout=0.5)
            # self.ser.bytesize = serial.EIGHTBITS
            # self.ser.timeout = 0.5

    def wait_until_light_ok(self):
        sleep_limit = 20
        sleep_count = 0
        while sleep_limit > sleep_count:
            read_content = self.ser.readline()
            rospy.loginfo("sleep count :{} read content : {}".format(sleep_count, read_content))
            if read_content == 'ok':
                return True
            else:
                sleep_count += 1

        rospy.loginfo('fail read ok')
        return False

    def control_light_open(self):
        rospy.loginfo('send message to control light')
        # for i in range(2)
        for i in range(2):
            self.ser.write([0xc4, 0xc4])
            rospy.sleep(0.5)

    def control_light_close(self):
        rospy.loginfo('send message to control light')
        # for i in range(2)
        for i in range(2):
            self.ser.write([0xc5, 0xc5])
            rospy.sleep(0.5)
        rospy.sleep(1)

    def control_humidifier_open(self):
        for i in range(2):
            self.ser.write([0xc2, 0xc2])
            rospy.sleep(0.5)

    def control_humidifier_close(self):
        for i in range(2):
            self.ser.write([0xc3, 0xc3])
            rospy.sleep(0.5)
        rospy.sleep(1)


if __name__ == '__main__':
    fan_contorl = Control('/dev/control', 9600, True)
    fan_contorl.control_light_open()
    fan_contorl.control_humidifier_open()
    rospy.sleep(6)
    fan_contorl.control_light_close()
    fan_contorl.control_humidifier_close()
