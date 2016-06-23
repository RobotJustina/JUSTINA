#!/usr/bin/env python
import rospy
import roslib
import cv2
import math
import sys
import copy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.srv import GetThermalAngle
from vision_msgs.srv import GetThermalAngleResponse

min_th = 140
max_th = 255

min_area = 1000
max_area = 50000

img_center = [165,128]
origin = [165,0]

angle_f = 0.0

vi = [img_center[0] - origin[0], img_center[1] - origin[1]]

def dotproduct(v1,v2):
    return sum((a*b) for a, b in zip(v1,v2))

def length(v):
    return math.sqrt(dotproduct(v,v))

def angle(v1,v2):
    return math.acos(dotproduct(v1,v2) / (length(v1)*length(v2)))

def get_angle(centers):

    if centers[0][0] > origin[0]:    
        v1 = [centers[0][0] - origin[0], centers[0][1] - origin[1]]
        return angle(v1,vi)*-1
    else:
        v1 = [origin[0] - centers[0][0], centers[0][1] - origin[1]]
        return angle(v1,vi)

def human_scaner(image):
    global flag
    centers = []

    clone = copy.copy(image)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(3,3))
    grayimg = clahe.apply(clone)

    grayimg = cv2.GaussianBlur(grayimg,(21,21),0)

    ret1,th1 = cv2.threshold(grayimg, min_th, 255,cv2.THRESH_BINARY)
    re2,th2 = cv2.threshold(grayimg, max_th, 255, cv2.THRESH_BINARY_INV)

    band_thresh = cv2.bitwise_and(th1, th2)

    contours, hierarchy = cv2.findContours(band_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea, reverse = True)[:1]

    for c in contours:
        if cv2.contourArea(c) > min_area and cv2.contourArea(c) < max_area: 

            M = cv2.moments(c)

            if M["m00"] != 0:
                cX = int(M["m10"]/ M["m00"])
                cY = int(M["m01"]/ M["m00"])

                centers.append((cX,cY))
                cv2.drawContours(clone, [c], -1, 0, 2)
                cv2.circle(clone, (cX, cY), 7, 0, 2)

    #cv2.imshow("band",clone)
    cv2.waitKey(1)
    if len(centers) != 0:
        angle_p = get_angle(centers)
        return angle_p
        
        #print "angulo en grados"
        #print math.degrees(angle_p)
    #else:
        #print ":("

def callback_2(data):
    global angle_f

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "mono8")
    cv_image = cv2.flip(cv_image,1)

    angle_f = human_scaner(cv_image)
    anglep = angle_f
    #print angle_f
    #return anglep


def callback(req):
    global angle_f
    #print "calculating_angle"
    flag = 0

    while flag < 1:
		#a = rospy.Subscriber("/hardware/thermal_camera/image_raw",Image,callback_2)
        a = rospy.Subscriber("/hardware/thermal_camera/image_raw",Image,callback_2)
        flag = flag+1

    print "responding..."
    return GetThermalAngleResponse(angle_f)

def main():
    global angle_f

    rospy.init_node('thermal_angle_server')
    s = rospy.Service('thermal_angle', GetThermalAngle, callback)
    print "Ready to get angle"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
