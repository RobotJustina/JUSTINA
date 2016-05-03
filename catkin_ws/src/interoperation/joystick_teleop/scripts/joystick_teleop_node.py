#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty

def callbackJoy(msg):
    global leftSpeed
    global rightSpeed
    global panPos
    global tiltPos
    global b_Button

    ### Read of b_button for stop the mobile base
    b_Button = msg.buttons[1]
    global stop

    ### Control of head with left Stick 
    leftStickX = msg.axes[0]
    leftStickY = msg.axes[1]

    ### Red button for stop of mobile base
    stop = msg.buttons[1]
    

    magnitudLeft = math.sqrt(leftStickX*leftStickX + leftStickY*leftStickY)
    if magnitudLeft > 0.1:
        panPos = leftStickX
        tiltPos = leftStickY
    else:
        panPos = 0
        tiltPos = 0
    

    ### Control of head with right Stick
    rightStickX = msg.axes[3]
    rightStickY = msg.axes[4]
    rightTrigger = msg.axes[5]
    magnitudRight = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitudRight > 0.1:
        turboFactor = 0.5 + (-rightTrigger + 1.0)/4.0
        leftSpeed = turboFactor*(rightStickY - 0.5*rightStickX)
        rightSpeed = turboFactor*(rightStickY + 0.5*rightStickX)
    else:
        leftSpeed = 0
        rightSpeed = 0
    
def main():
    global leftSpeed
    global rightSpeed
    global panPos 
    global tiltPos
    global b_Button
    global stop
    
    print "INITIALIZING JOYSTICK TELEOP BY MARCOSOFT..."
    rospy.init_node("joystick_teleop")
    
    # rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    pubSpeeds = rospy.Publisher("/hardware/mobile_base/speeds", Float32MultiArray, queue_size=1)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float32MultiArray, queue_size=1)
    pubStop = rospy.Publisher("/hardware/robot_state/stop", Empty, queue_size=1)
    pubStop = rospy.Publisher("/hardware/robot_state/stop", Empty, queue_size = 1)
    #pubHeadTorque = rospy.Publisher("/hardware/head/torque", Float32MultiArray, queue_size=1)
 

    loop = rospy.Rate(10)
    


    leftSpeed = 0
    rightSpeed = 0
    panPos = 0
    tiltPos = 0
    b_Button = 0
    stop = 0
    msgSpeeds = Float32MultiArray()
    msgHeadPos = Float32MultiArray()
    msgStop = Empty()
    #msgHeadTorque = Float32MultiArray()

    while not rospy.is_shutdown():
        if math.fabs(leftSpeed) > 0 or math.fabs(rightSpeed) > 0:
            msgSpeeds.data = [leftSpeed, rightSpeed]
            pubSpeeds.publish(msgSpeeds)


        if math.fabs(panPos) > 0 or math.fabs(tiltPos) > 0:
            msgHeadPos.data = [panPos, tiltPos]
            pubHeadPos.publish(msgHeadPos)

        if b_Button == 1:
            pubStop.publish(msgStop)

        #if math.fabs(panPos) > 0 or math.fabs(tiltPos) > 0:
            #msgHeadTorque.data = [panPos, tiltPos]
            #spubHeadTorque.publish(msgHeadTorque)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
