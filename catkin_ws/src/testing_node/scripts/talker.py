#!/usr/bin/env python
# license removed for brevity
import rospy
import rosgraph
import sys
import threading
from std_msgs.msg import String
    
def talker():
  pub = rospy.Publisher('chatter', String, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    if rosgraph.is_master_online():
      print "ROS MASTER is Online"
    else:
      print "ROS MASTER is Ofline"
    rate.sleep()
   
   #if __name__ == '__main__':
talker()
