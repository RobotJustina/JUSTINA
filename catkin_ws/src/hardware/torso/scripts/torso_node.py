#!/usr/bin/env python
import serial, time, sys, math
import rospy
import os
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
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

    jointStates = JointState()
    jointStates.name = ["spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0]

    pubTorsoPos = rospy.Publisher("/hardware/torso/current_pose",Float32MultiArray, queue_size = 1) 
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)

    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0,0,0]

    rate = rospy.Rate(30)
    if not simulated:
        print "Torso.-> Trying to open serial port on \"" + portName1 + "\""
        Roboclaw1.Open(portName1, 38400) #ttyACM0  --- M1: front  --- M2: rear
        address = 0x80
        print "Torso.-> Serial port openned on \"" + portName1 + "\" at 38400 bps (Y)"
        print "Torso.-> Clearing previous encoders readings"
        a, bumper , b = Roboclaw1.ReadEncM1(address)
        print "Torso.-> bumper ", bumper
        Roboclaw1.BackwardM2(address, 127) #Abajo-
        
        while bumper == 0 or bumper==1:
            a, bumper , b = Roboclaw1.ReadEncM1(address)
            print "Torso.->bumper ", bumper
        print "Torso.->bumper ", bumper
        Roboclaw1.BackwardM2(address, 0)
        torsoPos = 0
        Roboclaw1.SetEncM2(address, torsoPos)
        Roboclaw1.WriteNVM(address)

        Roboclaw1.ForwardM2(address, 127) #Arriba+
        a, torsoPos , b = Roboclaw1.ReadEncM2(address)
        a, bumper , b = Roboclaw1.ReadEncM1(address)
        while  torsoPos < 84866 or bumper==-1:
            a, torsoPos , b = Roboclaw1.ReadEncM2(address)
            a, bumper , b = Roboclaw1.ReadEncM1(address)
            print "Torso.->bumper ", bumper
        Roboclaw1.BackwardM2(address, 0)
        a, torsoPos , b = Roboclaw1.ReadEncM2(address)
        #print "Torso.-> MAX Torso Pos: ", torsoPos
        Roboclaw1.SetEncM2(address, torsoPos)
        Roboclaw1.WriteNVM(address)
        Roboclaw1.Close()
    else:
        torsoPos = 84866;

    msgCurrentPose.data[0] = (torsoPos*0.352)/169733 #-------------------pulsos a metros
    msgCurrentPose.data[1] = 0.0
    msgCurrentPose.data[2] = 0.0
    
    pubTorsoPos.publish(msgCurrentPose)
    jointStates.header.stamp = rospy.Time.now()
    jointStates.position = [(torsoPos*0.352)/169733, 0.0, 0.0, 0.0, 0.0]
    pubJointStates.publish(jointStates)
    



    


def callbackStop(msg):
    global stop
    stop = True

#169733------0.352

def callbackRelative(msg):
    global relH
    global stop
    global valueRel
    valueRel = True
    relH = (msg.data[0] * 169733)/0.352 ##Pasar de metros a pulsos
    stop = False

def callbackAbsolute(msg):
    global absH
    global stop
    global valueAbs
    valueAbs = True
    absH = (msg.data[0] * 169733)/0.352 ##Pasar de metros a pulsos
    stop = False 


def main(portName1, simulated):
    print "INITIALIZING TORSO..."


    ###Connection with ROS
    rospy.init_node("torso")

    jointStates = JointState()
    jointStates.name = ["spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0]

    pubTorsoPos = rospy.Publisher("/hardware/torso/current_pose",Float32MultiArray, queue_size = 1) 
    pubGoalReached = rospy.Publisher("/hardware/torso/goal_reached", Bool, queue_size=1)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    subRelativeHeight = rospy.Subscriber("/hardware/torso/goal_rel_pose",Float32MultiArray, callbackRelative)
    subAbsoluteHeight = rospy.Subscriber("/hardware/torso/goal_pose",Float32MultiArray, callbackAbsolute)
    subStop = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    
    rate = rospy.Rate(30)
    ###Communication with the Roboclaw
    global valueRel
    global valueAbs
    valueAbs = False
    valueRel = False
    torsoPos = 0
    bumper = 0;
    msgCurrentPose = Float32MultiArray()
    msgGoalReached = Bool()
    msgCurrentPose.data = [0,0,0]


    if not simulated:
        print "Torso.-> Trying to open serial port on \"" + portName1 + "\""
        Roboclaw1.Open(portName1, 38400) #ttyACM0  --- M1: front  --- M2: rear
        address = 0x80
        print "Torso.-> Serial port openned on \"" + portName1 + "\" at 38400 bps (Y)"
        print "Torso.-> Clearing previous encoders readings"
        a, torsoPos , b = Roboclaw1.ReadEncM2(address) 
        a, bumper , b = Roboclaw1.ReadEncM1(address) 
        
 	

    while not rospy.is_shutdown():
        #os.system('clear')
        initTorso = torsoPos
        if not simulated:
            if valueAbs and not stop:
                if absH > torsoPos:#sube
                    Roboclaw1.ForwardM2(address, 127) #Arriba+
                    while absH > torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                elif absH < torsoPos:#baja
                    Roboclaw1.BackwardM2(address, 127) #Abajo-
                    while absH < torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                valueAbs=False
            elif valueRel and not stop:
                absCalH = torsoPos + relH
                if absCalH > torsoPos:#sube
                    Roboclaw1.ForwardM2(address, 127) #Arriba+
                    while absCalH > torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                elif absCalH < torsoPos:#baja
                    Roboclaw1.BackwardM2(address, 127) #Abajo-
                    while absCalH < torsoPos and bumper == 0:
                         a, torsoPos , b = Roboclaw1.ReadEncM2(address)
                         a, bumper , b = Roboclaw1.ReadEncM1(address)
                    Roboclaw1.BackwardM2(address, 0) #Alto
                valueRel = False
            Roboclaw1.WriteNVM(address)
        else:
            if valueAbs and not stop:
                torsoPos=absH
                valueAbs=False
            elif valueRel and not stop:
                torsoPos = torsoPos + relH
                valueRel = False
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position = [(torsoPos*0.352)/169733, 0.0, 0.0, 0.0, 0.0]
        pubJointStates.publish(jointStates)



        msgCurrentPose.data[0] = (torsoPos*0.352)/169733 #-------------------pulsos a metros
        msgCurrentPose.data[1] = 0.0
        msgCurrentPose.data[2] = 0.0
        pubTorsoPos.publish(msgCurrentPose)
        msgGoalReached.data = abs(initTorso - torsoPos) < 200 
        pubGoalReached.publish(msgGoalReached)


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
