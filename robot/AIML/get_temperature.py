#!/usr/bin/python
# -*- coding: utf-8
import serial
import struct
# import rospy

def init_temperature():
    port = '/dev/control'
    baud = 9600
    ser = serial.Serial(port, baud)
    ser.bytesize = serial.EIGHTBITS
    ser.timeout = 0.5
    # if ser.is_open():
        # rospy.loginfo('serial success open')
    return ser
    # else:
    #     # rospy.loginfo('serial open failed')
    #     return None


def get_temperature(ser):
    # rospy.loginfo('start get temperature')
    # 读取数据命令
    # ser.write()
    ser.write([0xc1, 0xc1])
    content_read1 = ser.read(3)
    # content_read2 = ser.readline()
    # content_read = ser.readline()
    return content_read1


def main():
    ser = init_temperature()
    if not ser is None:
        data = get_temperature(ser)
        data = (bytes(data[2:]))
        data = struct.unpack('B', data)
        # print('{}'.format(data[0]))
        # print(bytes(data[2:]).decode('ascii'))
        print('当前的室内温度是{}度'.format(data[0]))
        ser.close()

if __name__ == '__main__':
    main()
