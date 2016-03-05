#!/usr/bin/env python
import rospy
import Dynamixel
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf

def printHelp():
    print "LEFT ARM NODE BY MARCOSOFT. Options:"

def main(portName1, portBaud1, portName2, portBaud2):
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("left_arm")
    br = tf.TransformBroadcaster()
    loop = rospy.Rate(10)
    ###Communication with dynamixels:
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)
    dynMan2 = Dynamixel.DynamixelMan(portName2, portBaud2)
    while not rospy.is_shutdown():
        pos0 = dynMan1.GetPresentPosition(0)
        pos1 = dynMan1.GetPresentPosition(1)
        pos2 = dynMan1.GetPresentPosition(2)
        print "Poses: " + str(pos0) + "   " + str(pos1) + "   " + str(pos2)
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName1 = "/dev/ttyUSB0"
            portName2 = "/dev/ttyUSB1"
            portBaud1 = 115200
            portBaud2 = 115200
            if "--port1" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--port2" in sys.argv:
                portName2 = sys.argv[sys.argv.index("--port2") + 1]
            if "--baud1" in sys.argv:
                portBaud1 = int(sys.argv[sys.argv.index("--baud1") + 1])
            if "--baud2" in sys.argv:
                portBaud2 = int(sys.argv[sys.argv.index("--baud2") + 1])
            main(portName1, portBaud1, portName2, portBaud2)
    except rospy.ROSInterruptException:
        pass
