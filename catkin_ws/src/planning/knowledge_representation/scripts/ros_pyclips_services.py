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

def spr_interpreter(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_int(req)
	return planning_cmdResponse(success, args)

def str_interpreter(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
        (success, args) = intSpeech.cmd_str_int(req.params)
        return planning_cmdResponse(success, args)

def interpreter_open(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
        (success, args) = intSpeech.cmd_int_open(req.params)
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

def ask_name(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success,args) = intSpeech.cmd_ask_name(req)
	return planning_cmdResponse(success, args)

def ask_incomplete(req):
	print "Receive: [%s %s]"%(req.name, req.params)
	(success,args) = intSpeech.cmd_ask_incomplete(req)
	return planning_cmdResponse(success, args)

def what_see(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.cmd_world(req)
    return planning_cmdResponse(success, args)

##OPEN CHALLENGE

def plan_explain(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.cmd_explain(req)
    return planning_cmdResponse(success, args)

def disponible(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.cmd_disp(req)
    return planning_cmdResponse(success, args)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)[('go to the bathroom and find the sponge', 0.99000001)]
    test = [(data.hypothesis[0], 0.99000001)]
    print "Texto Reconocido: " + data.hypothesis[0]
    intSpeech.subsRecSpeech(test)

def main():

    rospy.init_node('planning_clips_services')

    if "--mapping" in sys.argv:
        mappingName = sys.argv[sys.argv.index("--mapping") + 1]
    else:
        mappingName = "gpsr"

    intSpeech.set_mapping(mappingName)
    ######## servicios para los primeros pasos del interprete
    rospy.Service('/planning_clips/wait_command', planning_cmd, wait_command)
    rospy.Service('/planning_clips/spr_interpreter',planning_cmd, spr_interpreter)
    rospy.Service('/planning_clips/str_interpreter',planning_cmd, str_interpreter)
    rospy.Service('/planning_clips/interpreter_open',planning_cmd, interpreter_open)
    rospy.Service('/planning_clips/confirmation', planning_cmd, confirmation)
    rospy.Service('/planning_clips/get_task', planning_cmd, get_task)
    rospy.Service('/planning_clips/answer', planning_cmd, answer)
    rospy.Service('/planning_clips/ask_name', planning_cmd, ask_name)
    rospy.Service('/planning_clips/ask_incomplete', planning_cmd, ask_incomplete)

    #OPEN CHALLENGE
    rospy.Service('/planning_clips/what_see',planning_cmd,what_see)
    rospy.Service('/planning_clips/plan_explain',planning_cmd,plan_explain)
    rospy.Service('/planning_clips/disponible',planning_cmd,disponible)

    rospy.Subscriber("recognizedSpeech", RecognizedSpeech, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
