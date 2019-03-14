#!/usr/bin/env python

from std_msgs.msg import *
from manip_msgs.srv import *
import rospy
import numpy
import math

MAX_SERVO_SPEED = 12.26 #Max servo angular speed in rad/s

def callback_speed_profile(req):

    delta_t = req.dt
    t0 = req.t0
    tf = req.tf
    p0 = req.p0
    pf = req.pf
    v0 = req.w0
    vf = req.wf
    a0 = req.a0
    af = req.af

    A1 = [t0**5    ,  t0**4   , t0**3    , t0**2 , t0 ,  1 ]
    A2 = [5*t0**4  ,  4*t0**3 ,  3*t0**2 , 2*t0  , 1  ,  0 ]
    A3 = [20*t0**3 ,  12*t0**2 , 6*t0    , 2     , 0  ,  0 ]

    A4 = [tf**5    ,  tf**4   ,  tf**3   , tf**2 , tf ,  1 ]
    A5 = [5*tf**4  ,  4*tf**3 ,  3*tf**2 , 2*tf  , 1  ,  0 ]
    A6 = [20*tf**3 ,  12*tf**2 , 6*tf    , 2     , 0  ,  0 ]

    B = numpy.array([[p0],[v0],[a0],[pf],[vf],[af]])

    A = numpy.array([A1, A2, A3, A4, A5, A6])

    X = numpy.linalg.solve(A,B)

    n = int((math.fabs(tf - t0))/delta_t)


    positions = numpy.zeros(n)
    speeds = numpy.zeros(n)
    
    t = t0
    for i in range (n): 
        positions[i] = X[0]*t**5 + X[1]*t**4 + X[2]*t**3 + X[3]*t**2 + X[4]*t + X[5]
        speeds[i] = math.fabs(5*X[0]*t**4 + 4*X[1]*t**3 + 3*X[2]*t**2 + 2*X[3]*t + X[4] )/MAX_SERVO_SPEED
        t += delta_t

    res = SpeedProfileResponse()
    res.speeds= Float32MultiArray() 
    res.positions = Float32MultiArray()
    res.speeds.data = speeds
    res.positions.data = positions
    return res 

def speed_profile_sever():
    rospy.init_node('speed_profile_gen')
    print "Initializing speed_profile_gen"
    s = rospy.Service('/manipulation/get_speed_profile', SpeedProfile, callback_speed_profile )
    rospy.spin()

if __name__ == "__main__":
    speed_profile_sever()


