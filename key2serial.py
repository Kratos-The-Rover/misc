#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

class serial_commander():
    def __init__(self):
        
        self.key_sub = rospy.Subscriber('/keys',String, self.key_cb)
        self.rate = rospy.Rate(50)

    def key_cb(self, msg):
        key = msg.data
        my_serial.write(str(key))
        # my_serial.write('\n')
        print("in cb")
        line=my_serial.readline().decode("ascii")
        print(line)
        self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("serial_commander")
        my_serial = serial.Serial('/dev/ttyACM1',57600,timeout=1)

        commander = serial_commander()
        rospy.spin()

    except serial.serialutil.SerialException:
        print("device disconnected or already in use")
        
    except KeyboardInterrupt:
        my_serial.close()
