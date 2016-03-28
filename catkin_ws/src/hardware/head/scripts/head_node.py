#!/usr/bin/env python
import sys
import rospy
import Dynamixel
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf

def printHelp():
    print "HEAD NODE. Options:"
    print "TODO: Print all argument options"

def main(portName, portBaud):
    print "INITIALIZING HEAD NODE..."
    ###Connection with ROS
    rospy.init_node("head")
    br = tf.TransformBroadcaster()
    loop = rospy.Rate(10)
    ###Communication with dynamixels:
    dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)
    while not rospy.is_shutdown():
        bitsPerRadian = (1023)/((300)*(3.141592/180))
        #bitsPerRadian1 = (1023)/((300)*(3.141592/180))
        panPose = float((512-dynMan1.GetPresentPosition(5))/bitsPerRadian1)

        tiltPose = float((674-dynMan1.GetPresentPosition(1))/bitsPerRadian)
        
        print "Poses: " + str(panPose) + "   " + str(tiltPose)
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName = "/dev/ttyUSB2"
            portBaud = 1000000
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--baud" in sys.argv:
                portBaud = int(sys.argv[sys.argv.index("--baud") + 1])
            main(portName, portBaud)
    except rospy.ROSInterruptException:
        pass
