#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

def callbackJoy(msg):
    global leftSpeed
    global rightSpeed
    rightStickX = msg.axes[3]
    rightStickY = msg.axes[4]
    rightTrigger = msg.axes[5]
    magnitud = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitud > 0.1:
        turboFactor = 0.5 + (-rightTrigger + 1.0)/4.0
        leftSpeed = turboFactor*(rightStickY - 0.5*rightStickX)
        rightSpeed = turboFactor*(rightStickY + 0.5*rightStickX)
    else:
        leftSpeed = 0
        rightSpeed = 0
        

def main():
    print "INITIALIZING JOYSTICK TELEOP BY MARCOSOFT..."
    rospy.init_node("joystick_teleop")
    rospy.Subscriber("hardware/joy", Joy, callbackJoy)
    pubSpeeds = rospy.Publisher("hardware/mobile_base/speeds", Float32MultiArray, queue_size=1)
    loop = rospy.Rate(10)
    global leftSpeed
    global rightSpeed
    leftSpeed = 0
    rightSpeed = 0
    speeds = Float32MultiArray()
    while not rospy.is_shutdown():
        speeds.data = [leftSpeed, rightSpeed]
        pubSpeeds.publish(speeds)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
