#!/usr/bin/env python
import serial, time, sys, math
import rospy
import os
import comm
from datetime import datetime
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String
from sensor_msgs.msg import JointState


THR_DIFF_POS       = 2
MSG_MOTOR_TIMEOUT  = 1000000 #delay in microseconds 
MSG_SENSOR_TIMEOUT = 500000  #delay in microseconds 
DIST_LIM_INF       = 20.0
DIST_LIM_SUP       = 45.0
TORSO_ADJUSTMENT   = 0
ITER_RATE          = 30

def print_help():
    print "Torso. Options:"
    print "\t --port \t    Serial port name. If not provided, the default value is \"/dev/justinaTorso\""
    print "\t --simul\t    Simulation mode."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callbackEmergency(msg):
    global eme_stop
    global new_eme_msg_recv
    eme_stop = msg 
    new_eme_msg_recv = True

def callbackStop(msg):
    global stop
    stop = True

#169733------0.352

def callbackRelative(msg):
    global relH
    global stop
    global valueRel
    global simul

    valueRel = True
    if not simul:
        relH = msg.data[0]*100 ##Pasar de metros a pulsos
    else:
       relH = (msg.data[0]*169733)/0.352
       stop = False
       

def callbackAbsolute(msg):
    global absH
    global stop
    global valueAbs
    global simul

    valueAbs = True
    if not simul:
        absH = msg.data[0]*100 ##Pasar de metros a pulsos
    else:
        absH = (msg.data[0]*169733)/0.352
        stop = False
        

def callbackTorsoUp(msg):
    global torsoUp 
    global stop
    if not simul:
        torsoUp = True
    else:
        print "Torso Up simul"

def callbackTorsoDown(msg):
    global torsoDown 
    global stop
    if not simul:
        torsoDown = True
    else:
        print "Torso Down simul"

def callbackSimul(msg):
    global simul
    simul = msg.data 

def main():
    print "INITIALIZING TORSO..."
    global simul
    global pubGoalReached
    portName1 = "/dev/justinaTorso"

    ###Connection with ROS
    rospy.init_node("torso")

    if rospy.has_param('~simul'):
        simul = rospy.get_param('~simul')
    else:
        simul = True
   
    if rospy.has_param('~port'):
        portName1 = rospy.get_param('~port')
    elif not simul:
        print_help();
        sys.exit();

    jointStates = JointState()
    jointStates.name = ["spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0]

    pubTorsoPos       = rospy.Publisher ("/hardware/torso/current_pose",Float32MultiArray, queue_size = 1) 
    pubGoalReached    = rospy.Publisher ("/hardware/torso/goal_reached", Bool, queue_size=1)
    pubJointStates    = rospy.Publisher ("/joint_states", JointState, queue_size = 1)
    pubStop           = rospy.Publisher ("robot_state/stop", Empty, queue_size = 1)
    pubEmergencyStop  = rospy.Publisher ("robot_state/emergency_stop", Bool, queue_size = 1)
    subRelativeHeight = rospy.Subscriber("/hardware/torso/goal_rel_pose",Float32MultiArray, callbackRelative)
    subAbsoluteHeight = rospy.Subscriber("/hardware/torso/goal_pose",Float32MultiArray, callbackAbsolute)
    subStop           = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    subEmergencyStop  = rospy.Subscriber("robot_state/emergency_stop", Bool, callbackEmergency)
    subTorsoUp        = rospy.Subscriber("/hardware/torso/torso_up",String, callbackTorsoUp)
    subTorsoDown      = rospy.Subscriber("/hardware/torso/torso_down",String, callbackTorsoDown)
    subSimul          = rospy.Subscriber("/simulated", Bool, callbackSimul)
    
    rate = rospy.Rate(ITER_RATE)
    global valueRel
    global valueAbs
    global absH
    global relH
    global stop
    global torsoUp
    global torsoDown
    global eme_stop
    global new_eme_msg_recv

    valueAbs  = False
    valueRel  = False
    torsoUp   = False
    torsoDown = False
    torsoPos  = 0
    bumper    = 0
    stop      = False
    msgCurrentPose = Float32MultiArray()
    msgGoalReached = Bool()
    msgCurrentPose.data = [0,0,0]
    msgMotor = None
    initTimeMtrMsg = datetime.now()
    initTimeSnrMsg = datetime.now()
    timeoutSnr = 0
    timeoutMtr = 0
    if not simul:
        ArdIfc = comm.Comm(portName1)
        msgSensor = comm.Msg(comm.ARDUINO_ID, comm.MOD_SENSORS, comm.OP_GETCURRENTDIST, [], 0)
        ArdIfc.send(msgSensor)
    goalPose = 0
    new_eme_msg_recv = False 
    eme_stop = Bool() 
    eme_stop.data = False  

    while not rospy.is_shutdown():
        try:
            initTorso = torsoPos
            if not simul:
                timeoutSnr = datetime.now() - initTimeSnrMsg
                if timeoutSnr.microseconds > MSG_SENSOR_TIMEOUT:
                    ArdIfc.send(msgSensor)
                    initTimeSnrMsg = datetime.now()
                newMsg = ArdIfc.recv()
                if newMsg != None:
                    if newMsg.mod == comm.MOD_SENSORS: 
                        if newMsg.op == comm.OP_GETCURRENTDIST:
                            torsoPos = newMsg.param[0]
                            #rospy.loginfo("Torso-> Arduino ack GET CURRENT DIST msg received.")
                    if newMsg.mod == comm.MOD_SYSTEM: 
                        if newMsg.op == comm.OP_PING:
                            rospy.loginfo("Torso-> Arduino ack PING msg received.")
                        if newMsg.op == comm.OP_STOP:
                            rospy.loginfo("Torso-> Arduino Emercenty STOP system received.  ")
                            if eme_stop.data != bool(newMsg.param[0]):
                                eme_stop.data = newMsg.param[0]
                                pubEmergencyStop.publish(eme_stop)
                    if newMsg.mod == comm.MOD_MOTORS:
                        if newMsg.op == comm.OP_SETTORSOPOSE:
                            msgMotor_ack_received = True
                            rospy.loginfo("Torso-> Arduino ack SETTORSOPOSE msg received.")
                        if newMsg.op == comm.OP_GOUP:
                            msgMotor_ack_received = True
                            rospy.loginfo("Torso-> Arduino ack GOUP msg received.")
                        if newMsg.op == comm.OP_GODOWN:
                            msgMotor_ack_received = True
                            rospy.loginfo("Torso-> Arduino ack GODOWN msg received.")
                        if newMsg.op == comm.OP_STOP_MOTOR:
                            msgMotor_ack_received = True
                            #rospy.loginfo("Torso-> Arduino ack STOP MOTOR msg received.")

                #until ack received
                timeoutMtr = datetime.now() - initTimeMtrMsg
                if msgMotor != None and timeoutMtr.microseconds > MSG_MOTOR_TIMEOUT and not msgMotor_ack_received:
                     ArdIfc.send(msgMotor)
                     initTimeMtrMsg = datetime.now()
                
                if valueAbs and not eme_stop.data and absH >= DIST_LIM_INF and absH <= DIST_LIM_SUP:
                    msgMotor_ack_received = False
                    msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, int(absH), 1)
                    ArdIfc.send(msgMotor)
                    valueAbs=False
                    goalPose= absH
                    initTimeMtrMsg = datetime.now()
                elif valueRel and not eme_stop.data and (torsoPos + relH) >= DIST_LIM_INF and (torsoPos + relH) <= DIST_LIM_SUP:
                    msgMotor_ack_received = False
                    absCalH = torsoPos + relH
                    msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_SETTORSOPOSE, int(absCalH), 1)
                    ArdIfc.send(msgMotor)
                    goalPose=absCalH
                    valueRel = False
                    initTimeMtrMsg = datetime.now()
                elif ( valueAbs and (absH <  DIST_LIM_INF or absH > DIST_LIM_SUP) ) or ( valueRel and (torsoPos+relH > DIST_LIM_SUP or torsoPos+relH < DIST_LIM_INF) ):
                    rospy.logerr("Torso-> Can not reach te position.")
                    valueAbs = False
                    valueRel = False
                    goalPose= torsoPos
                elif torsoUp and not eme_stop.data:
                    rospy.loginfo("Torso-> Moving torso up.")
                    msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_GOUP, [], 0)
                    ArdIfc.send(msgMotor)
                    torsoUp = False
                    msgMotor_ack_received = False
                    initTimeMtrMsg = datetime.now()
                    goalPose= torsoPos
                elif torsoDown and not eme_stop.data:
                    rospy.loginfo("Torso-> Moving torso down.")
                    msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_GODOWN, [], 0)
                    ArdIfc.send(msgMotor)
                    torsoDown = False
                    msgMotor_ack_received = False 
                    initTimeMtrMsg = datetime.now()
                    goalPose= torsoPos
                elif eme_stop.data and new_eme_msg_recv:  
                    rospy.loginfo("Torso-> Stop message.")
                    msgMotor = comm.Msg(comm.ARDUINO_ID, comm.MOD_MOTORS, comm.OP_STOP_MOTOR, [], 0)
                    ArdIfc.send(msgMotor)
                    msgMotor_ack_received = False 
                    initTimeMtrMsg        = datetime.now()
                    new_eme_msg_recv      = False
                    torsoDown             = False 
                    torsoUp               = False 
                    valueAbs              = False 
                    valueRel              = False
                
                jointStates.header.stamp = rospy.Time.now()
                jointStates.position = [(torsoPos - TORSO_ADJUSTMENT)/100.0, 0.0, 0.0, 0.0, 0.0]
                pubJointStates.publish(jointStates)
                
                msgCurrentPose.data[0] = (torsoPos - TORSO_ADJUSTMENT) / 100.0
                msgCurrentPose.data[1] = 0.0
                msgCurrentPose.data[2] = 0.0
                pubTorsoPos.publish(msgCurrentPose)
                msgGoalReached.data = abs(goalPose - torsoPos) < THR_DIFF_POS 
                pubGoalReached.publish(msgGoalReached)
            else:
                if valueAbs and not stop:
                    torsoPos = absH
                    valueAbs = False
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
        except:
            rospy.logerr("Torso-> Oopps...we have some issues in this node :( ")    

    #End of while

#end of main()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
