#!/usr/bin/env python
import serial, time, sys, math
import rospy
import os
from std_msgs.msg import Float32
from hardware_tools import roboclaw_driver as Roboclaw1




def printHelp():
    print "Torso. Options:"
    print "\t --port \t    Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t    Simulation mode."
    print "\t -c, --calib  Calibration mode."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."


def calibration(portName1, simulated):
    print "INITIALIZING TORSO CALIBRATION..."
    rospy.init_node("torso")
    pubTorsoPos = rospy.Publisher("torso/position",Float32, queue_size = 1)
    rate = rospy.Rate(30)
    if not simulated:
        print "Torso.-> Trying to open serial port on \"" + portName1 + "\""
        Roboclaw1.Open(portName1, 38400) #ttyACM0  --- M1: front  --- M2: rear
        address = 0x80
        print "Torso.-> Serial port openned on \"" + portName1 + "\" at 38400 bps (Y)"
        print "Torso.-> Clearing previous encoders readings"
        a, bumper , b = Roboclaw1.ReadEncM1(address)
        Roboclaw1.BackwardM2(address, -127) #Abajo-
        while bumper == 0:
	        a, bumper , b = Roboclaw1.ReadEncM1(address)
        Roboclaw1.BackwardM2(address, 0)
        torsoPos = 0
        Roboclaw1.SetEncM2(address, torsoPos)#pasar torsoPos a pulsos
        Roboclaw1.WriteNVM(address)

        Roboclaw1.BackwardM2(address, 127) #Arriba+
        while bumper == 0:
	    	a, bumper , b = Roboclaw1.ReadEncM1(address)
	    Roboclaw1.BackwardM2(address, 0)
	    a, torsoPos , b = Roboclaw1.ReadEncM2(address)
	    print "Torso.-> MAX Torso Pos: "+ torsoPos
	    Roboclaw1.SetEncM2(address, torsoPos)
        Roboclaw1.WriteNVM(address)

        pubTorsoPos.publish(torsoPos)
        Roboclaw1.Close()


def callbackStop(msg):
    global stop
    stop = True



def callbackRelative(msg):
    global relH
    global stop
    global valueRel
    valueRel = True
    relH = msg.data ##Pasar de pulsos a metros
    stop = False

def callbackAbsolute(msg):
    global absH
    global stop
    global valueAbs
    valueAbs = True
    absH = msg.data ##Pasar de pulsos a metros
    stop = False 


def main(portName1, simulated):
    print "INITIALIZING TORSO..."


    ###Connection with ROS
    rospy.init_node("torso")
    pubTorsoPos = rospy.Publisher("torso/position",Float32, queue_size = 1) 
    subStop = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    subRelativeHeight = rospy.Subscriber("/hardware/torso/relative_height",Float32, callbackRelative)
    subAbsoluteHeight = rospy.Subscriber("/hardware/torso/absolute_height",Float32, callbackAbsolute)
    rate = rospy.Rate(30)
    ###Communication with the Roboclaw
    global valueRel
    global valueAbs
    valueAbs = False
    valueRel = False
    torsoPos =0;


    if not simulated:
        print "Torso.-> Trying to open serial port on \"" + portName1 + "\""
        Roboclaw1.Open(portName1, 38400) #ttyACM0  --- M1: front  --- M2: rear
        address = 0x80
        print "Torso.-> Serial port openned on \"" + portName1 + "\" at 38400 bps (Y)"
        print "Torso.-> Clearing previous encoders readings"
        a, torsoPos , b = Roboclaw1.ReadEncM2(address) ##Pasar de pulsos a metros
        a, bumper , b = Roboclaw1.ReadEncM1(address) 
        
 	

    while not rospy.is_shutdown():
        #os.system('clear')
        if not simulated:
            if valueAbs and not stop:
                if absH > torsoPos:#sube
                    Roboclaw1.BackwardM2(address, 127) #Arriba+
                    while absH > torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                elif absH < torsoPos:#baja
                    Roboclaw1.ForwardM2(address, -127) #Abajo-
                    while absH < torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                valueAbs=False
                Roboclaw1.WriteNVM(address)
                pubTorsoPos.publish(torsoPos)
            elif valueRel and not stop:
                absCalH = torsoPos + relH
                if absCalH > torsoPos:#sube
                    Roboclaw1.BackwardM2(address, 127) #Arriba+
                    while absCalH > torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                elif absCalH < torsoPos:#baja
                    Roboclaw1.ForwardM2(address, -127) #Abajo-
                    while absCalH < torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                valueRel = False
                Roboclaw1.WriteNVM(address)
                pubTorsoPos.publish(torsoPos)
        
        rate.sleep()
    #End of while
    

    if not simulated:
        Roboclaw1.Close()

#end of main()



if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        elif "--calib" in sys.argv:
            portName1 = "/dev/ttyACM0"
            simulated = False
            calibration(portName1, simulated)
        elif "-c" in sys.argv:
            portName1 = "/dev/ttyACM0"
            simulated = False
            calibration(portName1, simulated)
        else:
            portName1 = "/dev/ttyACM0"
            simulated = False
            if "--port" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName1, simulated)
    except rospy.ROSInterruptException:
        pass
