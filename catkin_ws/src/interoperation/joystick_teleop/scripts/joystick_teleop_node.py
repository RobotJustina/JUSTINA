#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def callbackJoy(msg):
    global speedX
    global speedY
    global yaw
    global panPos
    global tiltPos
    global b_Button

    ### Read of b_button for stop the mobile base
    b_Button = msg.buttons[1]
    global stop

    ### Control of head with left Stick 
    leftStickX = msg.axes[0]
    leftStickY = msg.axes[1]

    if msg.axes[2] == 0 and msg.axes[5] != 0:
        leftTigger = msg.axes[2]
        rightTigger = -(msg.axes[5] - 1) 

    elif msg.axes[5] == 0 and msg.axes[2] != 0:
        leftTigger = -(msg.axes[2] - 1)
        rightTigger = msg.axes[5] 

    elif msg.axes[5] == 0 and msg.axes[2] == 0:
        leftTigger = 0
        rightTigger = 0 
    else:
        leftTigger = -(msg.axes[2] - 1)
        rightTigger = -(msg.axes[5] - 1) 


    #print "leftTigger: " + str(leftTigger) +" rightTigger: " + str(rightTigger)

    ### Red button for stop of mobile base
    stop = msg.buttons[1]

    ### Tigger button for speed y componente

    magnitudTiggerDiference = math.sqrt((leftTigger*leftTigger) + (rightTigger*rightTigger))
    #print "diference: " + str(magnitudTiggerDiference)
    if magnitudTiggerDiference > 0.25:
        speedY = (leftTigger - rightTigger)/2
    else:
        speedY = 0

    magnitudLeft = math.sqrt(leftStickX*leftStickX + leftStickY*leftStickY)
    if magnitudLeft > 0.1:
        panPos = leftStickX
        tiltPos = leftStickY
    else:
        panPos = 0
        tiltPos = 0
    

    ### Control of mobile-base with right Stick
    rightStickX = msg.axes[3]
    rightStickY = msg.axes[4]
    magnitudRight = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitudRight > 0.1:
        speedX = rightStickY
        yaw = rightStickX
    else:
        speedX = 0
        yaw = 0
    
def main():
    global leftSpeed
    global rightSpeed
    global panPos 
    global tiltPos

    global speedX
    global speedY
    global yaw
    global stop
    

    leftSpeed = 0
    rightSpeed = 0
    panPos = 0
    tiltPos = 0
    b_Button = 0
    stop = 0
    speedY = 0
    speedX = 0
    yaw = 0

    msgSpeeds = Float32MultiArray()
    msgHeadPos = Float32MultiArray()
    msgTwist = Twist()
    msgStop = Empty()
    #msgHeadTorque = Float32MultiArray()
    
    print "INITIALIZING JOYSTICK TELEOP BY MARCOSOFT... :)"
    rospy.init_node("joystick_teleop")
    
    # rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float32MultiArray, queue_size=1)
    pubStop = rospy.Publisher("/hardware/robot_state/stop", Empty, queue_size = 1)
    pubTwist = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size =1)
    #pubHeadTorque = rospy.Publisher("/hardware/head/torque", Float32MultiArray, queue_size=1)

    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        if math.fabs(speedX) > 0 or math.fabs(speedY) > 0 or math.fabs(yaw) > 0:
            msgTwist.linear.x = speedX/1
            msgTwist.linear.y = speedY/2.0
            msgTwist.linear.z = 0
            msgTwist.angular.z = yaw/2.0
            #print "x: " + str(msgTwist.linear.x) + "  y: " + str(msgTwist.linear.y) + " yaw: " + str(msgTwist.angular.z)
            pubTwist.publish(msgTwist)

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
