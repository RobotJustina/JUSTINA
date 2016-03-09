#!/usr/bin/env python
import sys
import rospy
import Dynamixel
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf

def printHelp():
    print "LEFT ARM NODE BY MARCOSOfT. Options:"
    
def main(portName1, portBaud1, portName2, portBaud2):
    print "INITIALIZING MOBILE BASE (Edgar version)..."
    ###Connection with ROS
    rospy.init_node("left_arm")
    br = tf.TransformBroadcaster()
    loop = rospy.Rate(10)
    ###Communication with dynamixels:
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)
    dynMan2 = Dynamixel.DynamixelMan(portName2, portBaud2)
    while not rospy.is_shutdown():
        pos0 = dynMan1.GetPresentPosition(0)
        pos0 = ((2094-pos0)/(4095))*(3.141592)

        pos1 = dynMan1.GetPresentPosition(1)
        pos1 = ((3127-pos1)/(4095))*(3.141592)
        
        pos2 = dynMan1.GetPresentPosition(2)
        pos2 = ((1798-pos2)/(4095))*(3.141592)

        pos3 = dynMan1.GetPresentPosition(3)
        pos3 = ((1997-pos3)/(4095))*(3.141592)

        pos4 = dynMan1.GetPresentPosition(4)
        pos4 = ((2050-pos4)/(4095))*(3.141592)
        
        pos5 = dynMan1.GetPresentPosition(5)
        pos5 = ((1774-pos5)/(4095))*(3.141592)
        
        pos6 = dynMan1.GetPresentPosition(6)
        pos6 = ((2048-pos6)/(4095)*(3.141592)
        
        pos7 = dynMan2.GetPresentPosition(7)
        pos7 = ((512-pos7)/(1023))*(3.141592)
        
        pos8 = dynMan2.GetPresentPosition(107)
        pos8 = ((512-pos8)/(1023))*(3.141592)
        
        print "Poses: " + str(pos0) + "  " + str(pos1) + "  " + str(pos2) + "  " + str(pos3)
             + "  " + str(pos4) + "  " + str(pos5) + "  " + str(pos6) + "  " + str(pos7) + "  " + str(pos8)
             
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
