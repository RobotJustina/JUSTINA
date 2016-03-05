#!/usr/bin/env python
import rospy

#!/usr/bin/env python
import serial, time, sys, math
import rospy
import Roboclaw
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf

def printHelp():
    print "LEFT ARM NODE BY MARCOSOFT. Options:"
    

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName1 = "/dev/ttyUSB0"
            portName2 = "/dev/ttyUSB1"
            if "--port1" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--port2" in sys.argv:
                portName2 = sys.argv[sys.argv.index("--port2") + 1]
            main(portName1, portName2)
    except rospy.ROSInterruptException:
        pass
