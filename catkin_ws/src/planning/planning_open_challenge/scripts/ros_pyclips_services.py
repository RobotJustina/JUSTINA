#!/usr/bin/env python
from knowledge_msgs.msg import *
from knowledge_msgs.srv import *
from interprete import intSpeech

import rospy

#Service for the task.
def wait_command(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_speech(req)
	return planning_cmdResponse(success, args)

def interpreter(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_int(req)
	return planning_cmdResponse(success, args)

def confirmation(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_conf(req)
	return planning_cmdResponse(success, args)

def get_task(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_task(req)
	return planning_cmdResponse(success, args)

def answer(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.answer(req)
    return planning_cmdResponse(success, args)

def what_see(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.cmd_world(req)
    return planning_cmdResponse(success, args)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)[('go to the bathroom and find the sponge', 0.99000001)]
    test = [(data.hypothesis[0], 0.99000001)]
    print "Texto Reconocido: " + data.hypothesis[0]
    intSpeech.subsRecSpeech(test)

def plan_explain(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.cmd_explain(req)
    return planning_cmdResponse(success, args)

def disponible(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.cmd_disp(req)
    return planning_cmdResponse(success, args)

def main():

    rospy.init_node('planning_clips_services')
    
    ######## servicios para los primeros pasos del interprete
    rospy.Service('/planning_open_challenge/wait_command', planning_cmd, wait_command)
    rospy.Service('/planning_open_challenge/interpreter',planning_cmd, interpreter)
    rospy.Service('/planning_open_challenge/confirmation', planning_cmd, confirmation)
    rospy.Service('/planning_open_challenge/get_task', planning_cmd, get_task)
    rospy.Service('/planning_open_challenge/answer', planning_cmd, answer)
    rospy.Service('/planning_open_challenge/what_see',planning_cmd,what_see)
    rospy.Service('/planning_open_challenge/plan_explain',planning_cmd,plan_explain)
    rospy.Service('/planning_open_challenge/disponible',planning_cmd,disponible)

    rospy.Subscriber("recognizedSpeech", RecognizedSpeech, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
