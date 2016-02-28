#!/usr/bin/env python
import rospy

def main():
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    rospy.init_node("mobile_base")

if __name__ == '__main__':
    try:
        main()
    except:
        pass
