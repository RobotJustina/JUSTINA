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
    print "\t --port \t    Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
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
    print 'In callbackAboslute'
    valueAbs = True
    absH = msg.data[0]*100 ##Pasar de metros a pulsos
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
    global absH
    global relH
    global stop
    valueAbs = False
    valueRel = False
    torsoPos = 0
    bumper = 0;
    msgCurrentPose = Float32MultiArray()
    msgGoalReached = Bool()
    msgCurrentPose.data = [0,0,0]


          
    #if not simulated:
    ArdIfc = comm.Comm(portName1)
    msgSensor = comm.Msg(comm.ARDUINO_ID,comm.MOD_SENSORS,comm.OP_GETCURRENTDIST,[],0)
    #msgSensor = comm.Msg(comm.ARDUINO_ID,comm.MOD_SYSTEM,comm.OP_PING,[],0)
    

    while not rospy.is_shutdown():
       
        if not simulated:
            ArdIfc.send(msgSensor)

            newMsg = ArdIfc.recv()
            if newMsg != None:
            	
            	if newMsg.mod == comm.MOD_SENSORS: 
            		if newMsg.op == comm.OP_GETCURRENTDIST:
                		torsoPos = newMsg.param[0]
                if newMsg.mod == comm.MOD_SYSTEM: 
            		if newMsg.op == comm.OP_PING:
                		#print "Ping Ok"
				if newMsg.mod == comm.MOD_MOTORS: 
					if newMsg.op == comm.OP_SETTORSOPOSE:
						#print newMsg.param[0]
        initTorso = torsoPos
        if not simulated:
            if valueAbs and  not stop and absH > 20.0 and absH < 50.0 :
            	
                msgMotor = comm.Msg(comm.ARDUINO_ID,comm.MOD_MOTORS,comm.OP_SETTORSOPOSE,int(absH),1)
                ArdIfc.send(msgMotor)
                valueAbs=False
            elif valueRel and not stop and torsoPos+relH > 20.0 and torsoPos+relH < 50.0:
                absCalH = torsoPos + relH
                msgMotor = comm.Msg(comm.ARDUINO_ID,comm.MOD_MOTORS,comm.OP_SETTORSOPOSE,int(absCalH),1)
                ArdIfc.send(msgMotor)
                valueRel = False
            elif absH < 20.0 or absH > 50.0 or torsoPos+relH > 50.0 or torsoPos+relH < 20.0:
            	rospy.logerr("Torso-> Can not reach te position.")
            	valueAbs = False
            	valueRel = False
        else:
            if valueAbs and not stop:
                torsoPos=absH
                valueAbs=False
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
    #End of while
    

  

#end of main()



if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
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
