#!/usr/bin/env python

import serial, time, sys, math
import rospy
import os
import comm
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from hardware_tools import roboclaw_driver as Roboclaw1


THR_DIFF_POS = 2



def printHelp():
    print "Torso. Options:"
    print "\t --port \t    Serial port name. If not provided, the default value is \"/dev/justinaTorso\""
    print "\t --simul\t    Simulation mode."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callbackStop(msg):
    global stop
    stop = True

#169733------0.352

def callbackRelative(msg):
    global relH
    global stop
    global valueRel
    valueRel = True
    relH = msg.data[0]*100 ##Pasar de metros a pulsos
    stop = False

def callbackAbsolute(msg):
    global absH
    global stop
    global valueAbs
    valueAbs = True
    absH = msg.data[0]*100 ##Pasar de metros a pulsos
    stop = False 

def callbackTorsoUp(msg):
    global torsoUp 
    global stop
    torsoUp = True
    stop = False 

def callbackTorsoDown(msg):
    global torsoDown 
    global stop
    torsoDown = True 
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
    subTorsoUp = rospy.Subscriber("/hardware/torso/torso_up",std_msgs::String, callbackTorsoUp)
    subTorsoDown = rospy.Subscriber("/hardware/torso/torso_down",std_msgs::String, callbackTorsoDown)
    
    rate = rospy.Rate(30)
    ###Communication with the Roboclaw
    global valueRel
    global valueAbs
    global absH
    global relH
    global stop
    global torsoUp
    global torsoDown

    valueAbs = False
    valueRel = False
    torsoUp = False
    torsoDown = False
    torsoPos = 0
    bumper = 0;
    msgCurrentPose = Float32MultiArray()
    msgGoalReached = Bool()
    msgCurrentPose.data = [0,0,0]
    #absH = 10  #just for compoiling - FIXME:do it better
    #relH = 100 #just for compoiling - FIXME:do it better
    msgMotor = None

    #connectionError = True
    #if not simulated:
    #while connectionError: 
    #    print "Trying to connect to arduino board..."
    #    try:
    #        ArdIfc = comm.Comm(portName1)            
    #    except Exception, e:
    #        print "Comm: Error al intentar conectarse"    
    #    time.sleep(1)
    #print "salio del while"
    ArdIfc = comm.Comm(portName1)
    #msgSensor = comm.Msg(comm.ARDUINO_ID, comm.MOD_SENSORS, comm.OP_GETCURRENTDIST, [], 0)
    #msgSensor = comm.Msg(comm.ARDUINO_ID, comm.MOD_SYSTEM, comm.OP_PING,[],0)
    msgSensor = comm.Msg( comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, 30, 1)

    while not rospy.is_shutdown():
        simulated  = False
        if not simulated:
            ArdIfc.send(msgSensor)
            newMsg = ArdIfc.recv()
            if newMsg != None:
            	if newMsg.mod == comm.MOD_SENSORS: 
            		if newMsg.op == comm.OP_GETCURRENTDIST:
                		torsoPos = newMsg.param[0]
                        #print torsoPos
                if newMsg.mod == comm.MOD_SYSTEM: 
            		if newMsg.op == comm.OP_PING:
                		print "Ping Ok"
		if newMsg.mod == comm.MOD_MOTORS:
			if newMsg.op == comm.OP_SETTORSOPOSE:
				msgMotor_ack_received = True

        #until ack received
        if msgMotor != None and not msgMotor_ack_received:
             ArdIfc.send(msgMotor)
             print "resending msgMotor"

        initTorso = torsoPos
        if not simulated:
            if valueAbs and  not stop and absH > 20.0 and absH < 50.0:
                print "in abs send"
                msgMotor_ack_received = False
                #msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, int(absH), 1)
                msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, 30, 1)
                ArdIfc.send(msgMotor)
                valueAbs=False
            elif valueRel and not stop and torsoPos+relH > 20.0 and torsoPos+relH < 50.0:
                absCalH = torsoPos + relH
                msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, int(absCalH), 1)
                ArdIfc.send(msgMotor)
                valueRel = False
            elif ( valueAbs and (absH < 20.0 or absH > 50.0) ) or ( valueRel and (torsoPos+relH > 50.0 or torsoPos+relH < 20.0) ):
            	rospy.logerr("Torso-> Can not reach te position.")
            	valueAbs = False
            	valueRel = False
            elif torsoUp and not stop:
            	rospy.loginfo("Torso-> Moving torso up.")
                msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_GOUP, [], 0)
                ArdIfc.send(msgMotor)
            	torsoUp = False
		msgMotor_ack_received = False
            elif torsoDown and not stop:
            	rospy.loginfo("Torso-> Moving torso down.")
                msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, int(absCalH), 1)
                ArdIfc.send(msgMotor)
            	torsoDown = False
		msgMotor_ack_received = True 
        else:
            if valueAbs and not stop:
                torsoPos = absH
                valueAbs = False
            elif valueRel and not stop:
                torsoPos = torsoPos + relH
                valueRel = False
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position = [torsoPos, 0.0, 0.0, 0.0, 0.0]
        pubJointStates.publish(jointStates)
        
        msgCurrentPose.data[0] = torsoPos/100.0
        msgCurrentPose.data[1] = 0.0
        msgCurrentPose.data[2] = 0.0
        pubTorsoPos.publish(msgCurrentPose)
        msgGoalReached.data = abs(initTorso - torsoPos) < THR_DIFF_POS 
        pubGoalReached.publish(msgGoalReached)

        rate.sleep()
        time.sleep(0.1)             #FIXME:do we have to put some delay here
    #End of while

#end of main()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName1 = "/dev/justinaTorso"
            simulated = False
            if "--port" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName1, simulated)
    except rospy.ROSInterruptException:
        pass
