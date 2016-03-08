#!/usr/bin/env python
import sys, time, math
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def printHelp():
    print "MOBILE BASE SIMULATED BY MARCOSOFT. Options:"
    print "TODO: Print all options"

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global newSpeedData
    #Speeds are assumed to come in float in [-1,1] for each tire. The value need to be transformed to values in [0,127]
    #A float value of -1, indicates the maximum speed backwards. Similar for +1
    leftSpeed = int(msg.data[0] * 127)
    rightSpeed = int(msg.data[1] * 127)
    newSpeedData = True

def calculateOdometry(currentPos, leftEnc, rightEnc):
    #In the node for the real robot, leftEnc and rightEnc are assumed to come in ticks.
    #In this node, such distances are assumed to expressed in meters
    deltaTheta = (rightEnc - leftEnc)/0.48
    if math.fabs(deltaTheta) >= 0.0001:
        rg = (leftEnc + rightEnc)/(2*deltaTheta)
        deltaX = rg*math.sin(deltaTheta)
        deltaY = rg*(1-math.cos(deltaTheta))
    else:
        deltaX = (leftEnc + rightEnc)/2
        deltaY = 0
    currentPos[0] += deltaX*math.cos(currentPos[2]) - deltaY*math.sin(currentPos[2])
    currentPos[1] += deltaX*math.sin(currentPos[2]) + deltaY*math.cos(currentPos[2])
    currentPos[2] += deltaTheta
    return currentPos

def main():
    print "INITIALIZING MOBILE BASE SIMULATED BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("mobile_base")
    pubOdometry = rospy.Publisher("mobile_base/odometry", Odometry, queue_size=1)
    subSpeeds = rospy.Subscriber("mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10):
    
