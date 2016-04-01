#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

def callbackJoy(msg):
    global leftSpeed
    global rightSpeed
    global panPos
    global tiltPos

    ### Control of head with left Stick 
    leftStickX = msg.axes[0]
    leftStickY = msg.axes[1]
    magnitudLeft = math.sqrt(leftStickX*leftStickX + leftStickY*leftStickY)
    if magnitudLeft > 0.1:
        panPos = leftStickX
        tiltPos = leftStickY
    else:
        panPos = 0
        tiltPos = 0
    
    # print str(panPos) + "  " + str(tiltPos)


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
    
    # print str(leftSpeed) + "  " + str(rightSpeed)

def main():
    print "INITIALIZING JOYSTICK TELEOP BY MARCOSOFT..."
    rospy.init_node("joystick_teleop")
    
    # rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    rospy.Subscriber("/joy", Joy, callbackJoy)
    pubSpeeds = rospy.Publisher("/hardware/mobile_base/speeds", Float32MultiArray, queue_size=1)
    pubHeadPos = rospy.Publisher("/goal_pose", Float32MultiArray, queue_size=1)
    loop = rospy.Rate(10)
    
    global leftSpeed
    global rightSpeed
    global panPos 
    global tiltPos

    leftSpeed = 0
    rightSpeed = 0
    panPos = 0
    tiltPos = 0
    speeds = Float32MultiArray()
    headPos = Float32MultiArray()

    while not rospy.is_shutdown():
        speeds.data = [leftSpeed, rightSpeed]
        headPos.data = [panPos, tiltPos]
        pubHeadPos.publish(headPos)
        pubSpeeds.publish(speeds)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
