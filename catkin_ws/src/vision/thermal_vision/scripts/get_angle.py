#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import math
import copy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

min_th = 140
max_th = 255

min_area = 1000
max_area = 50000

img_center = [165,128]
origin = [165,0]

vi = [img_center[0] - origin[0], img_center[1] - origin[1]]

flag = 0

def dotproduct(v1,v2):
    return sum((a*b) for a, b in zip(v1,v2))

def length(v):
    return math.sqrt(dotproduct(v,v))

def angle(v1,v2):
    return math.acos(dotproduct(v1,v2) / (length(v1)*length(v2)))

def get_angle(centers):

    if centers[0][0] > origin[0]:    
        v1 = [centers[0][0] - origin[0], centers[0][1] - origin[1]]
        return angle(v1,vi)
    else:
        v1 = [origin[0] - centers[0][0], centers[0][1] - origin[1]]
        return angle(v1,vi)*-1
    
    #v2 = [centers[1][0] - origin[0], centers[1][1] - origin[1]]

def human_scaner(image):
    global i, flag
    centers = []

    clone = copy.copy(image)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(3,3))
    grayimg = clahe.apply(clone)

    grayimg = cv2.GaussianBlur(grayimg,(21,21),0)

    ret1,th1 = cv2.threshold(grayimg, min_th, 255,cv2.THRESH_BINARY)
    re2,th2 = cv2.threshold(grayimg, max_th, 255, cv2.THRESH_BINARY_INV)

    band_thresh = cv2.bitwise_and(th1, th2)

    cv2.imshow("band", band_thresh)
    cv2.moveWindow("band",450,0)

    contours, hierarchy = cv2.findContours(band_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea, reverse = True)[:1]

    #print 'Contornos encontrados'
    #print len(contours)

    for c in contours:
        if cv2.contourArea(c) > min_area and cv2.contourArea(c) < max_area: 

            i=i+1
            M = cv2.moments(c)

            if M["m00"] != 0:
                cX = int(M["m10"]/ M["m00"])
                cY = int(M["m01"]/ M["m00"])

                centers.append((cX,cY))
                #print centers
              
                #xA = cX - w_width/2
                #yA = cY - w_height/2

                #xB = cX + w_width/2
                #yB = cY + w_height/2

                #cv2.rectangle(clone, (xA,yA),(xB,yB),0,1)
                #aux_roi = clone[yA:yB,xA:xB]
                #pre_rois.append(aux_roi)

                cv2.drawContours(clone, [c], -1, 0, 2)
                cv2.circle(clone, (cX, cY), 7, 0, 2)
                #print "iteracion"
                #print i

    cv2.imshow("result", clone)
    cv2.moveWindow("result",0,400)
    cv2.waitKey(1)

    if len(centers) != 0:
        angle_p = get_angle(centers)
        #print "angulo en radian"
        #print angle_p

        while flag < 1000:
            pub = rospy.Publisher('thermal_angle', String, queue_size=10)
            pub.publish(str(angle_p))
            print "angulo en radian"
            print angle_p
            flag = flag + 1
            print flag

        print "angulo en grados"
        print math.degrees(angle_p)
    else:
        print "no hay blobs"

def callback(data):
  global i
  
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(data, "mono8")
  cv_image = cv2.flip(cv_image,1)
  cv2.imshow("Original_Image", cv_image)
  cv2.moveWindow("Original_Image",0,0)
  human_scaner(cv_image)

def main(args):
  global i
  
  i = 0

  rospy.init_node('get_angle', anonymous=False)
  #image_sub = rospy.Subscriber("camera/image",Image,callback)
  image_sub = rospy.Subscriber("thermal_camera/image_raw",Image,callback)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)