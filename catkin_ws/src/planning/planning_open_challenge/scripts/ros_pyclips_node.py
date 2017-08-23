#!/usr/bin/env python
import time, threading, os
import Tkinter as tk

import clipsFunctions
from clipsFunctions import clips

import pyRobotics.BB as BB
from pyRobotics.Messages import Command, Response

from knowledge_msgs.msg import *
from knowledge_msgs.srv import *
from interprete import intSpeech

import BBFunctions

import rospy

defaultTimeout = 2000
defaultAttempts = 1

def callbackCommandResponse(data):
    print "callbackCommandResponse name command:" + data.name
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    clipsFunctions.Assert('(BB_received "{0}" {1} {2} "{3}")'.format(data.name, data.id, data.successful, data.params))
    clipsFunctions.PrintOutput()
    clipsFunctions.Run(BBFunctions.gui.getRunTimes())
    clipsFunctions.PrintOutput()
    
def setCmdTimer(t, cmd, cmdId):
    t = threading.Thread(target=cmdTimerThread, args = (t, cmd, cmdId))
    t.daemon = True
    t.start()
    return True

def cmdTimerThread(t, cmd, cmdId):
    time.sleep(t/1000)
    clipsFunctions.Assert('(BB_timer "{0}" {1})'.format(cmd, cmdId))
    clipsFunctions.PrintOutput()
    #clipsFunctions.Run(gui.getRunTimes())
    #clipsFunctions.PrintOutput()

def setTimer(t, sym):
    t = threading.Thread(target=timerThread, args = (t, sym))
    t.daemon = True
    t.start()
    return True

def timerThread(t, sym):
    time.sleep(t/1000)
    clipsFunctions.Assert('(BB_timer {0})'.format(sym))
    clipsFunctions.PrintOutput()
    #clipsFunctions.Run(gui.getRunTimes())
    #clipsFunctions.PrintOutput()

def SendCommand(cmdName, params, timeout = defaultTimeout, attempts = defaultAttempts):
    global pubUnknown
    print 'Function name ' + cmdName
    cmd = Command(cmdName, params)
    func = fmap.get(cmdName)
    if func != None:
        func(cmd)
    else:
        request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
        pubUnknown.publish(request)
    return cmd._id

#def SendResponse(cmdName, cmd_id, result, response):
    #result = str(result).lower() not in ['false', '0']
    #r = Response(cmdName, result, response)
    #r._id = cmd_id
    #BB.Send(r)

def Initialize():
    clips.Memory.Conserve = True
    clips.Memory.EnvironmentErrorsEnabled = True
    
    clips.RegisterPythonFunction(SendCommand)
    #clips.RegisterPythonFunction(SendResponse)
    clips.RegisterPythonFunction(setCmdTimer)
    clips.RegisterPythonFunction(setTimer)
    #clips.RegisterPythonFunction(CreateSharedVar)
    #clips.RegisterPythonFunction(WriteSharedVar)
    #clips.RegisterPythonFunction(SubscribeToSharedVar)
    
    clips.BuildGlobal('defaultTimeout', defaultTimeout)
    clips.BuildGlobal('defaultAttempts', defaultAttempts)
    
    filePath = os.path.dirname(os.path.abspath(__file__))
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'BB_interface.clp')
    
    file_gpsr = filePath + '/open_challenge/challenge.dat'
    print file_gpsr
    BBFunctions.gui.putFileName(file_gpsr)
    BBFunctions.gui.loadFile()
    BBFunctions.gui.reset()

#Funcions to fmap, this functions are publish to topics to do the tasks
def cmd_speech(cmd):
    global pubCmdSpeech
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdSpeech.publish(request)
    return cmd._id

def cmd_int(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdInt.publish(request)
    return cmd._id

def cmd_conf(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdConf.publish(request)
    return cmd._id

def cmd_task(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdGetTask.publish(request)
    return cmd._id

def goto(cmd):
    global pubCmdGoto
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdGoto.publish(request)
    return cmd._id

def answer(cmd):
    global pubCmdAnswer
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAnswer.publish(request)
    return cmd._id

def find_object(cmd):
    global pubCmdFindObject
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdFindObject.publish(request)
    return cmd._id

def ask_for(cmd):
    global pubCmdAskFor
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAskFor.publish(request)
    return cmd._id

def status_object(cmd):
    global pubCmdStatusObject
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdStatusObject.publish(request)
    return cmd._id

def move_actuator(cmd):
    global pubCmdMoveActuator
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdMoveActuator.publish(request)
    return cmd._id

def drop(cmd):
    global pubDrop
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubDrop.publish(request)
    return cmd._id

def cmd_world(cmd):
    global pubCmdWorld
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdWorld.publish(request)
    return cmd._id

def cmd_describe(cmd):
    global pubCmdDescribe
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdDescribe.publish(request)
    return cmd._id

def cmd_where(cmd):
    global pubCmdWhere
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdWhere.publish(request)
    return cmd._id

def cmd_order(cmd):
    global pubCmdTakeOrder
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdTakeOrder.publish(request)
    return cmd._id

def cmd_explain(cmd):
    global pubCmdExplain
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdExplain.publish(request)
    return cmd._id

def cmd_disp(cmd):
    global pubCmdDisp
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdDisp.publish(request)
    return cmd._id

def cmd_happen(cmd):
    global pubCmdHappen
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdHappen.publish(request)
    return cmd._id


#Define the function map, this function are the functions that represent of task in the clips rules.
fmap = {
    'cmd_speech': cmd_speech,
    'cmd_int': cmd_int,
    'cmd_conf': cmd_conf,
    'cmd_task': cmd_task,
    'find_object': find_object,
    'move_actuator': move_actuator,
    #'grab': grab,
    'drop': drop,
    'status_object': status_object,
    'goto': goto,
    #'speak': speak,
    'ask_for': ask_for,
    'answer' : answer,
    'cmd_world':cmd_world,
    'cmd_describe':cmd_describe,
    'cmd_where':cmd_where,
    'cmd_order':cmd_order,
    'cmd_explain':cmd_explain,
    'cmd_disp':cmd_disp,
    'cmd_happen':cmd_happen
}

def main():

    global pubCmdSpeech, pubCmdInt, pubCmdConf, pubCmdGetTask, pubUnknown
    global pubCmdGoto, pubCmdAnswer, pubCmdFindObject, pubCmdAskFor, pubCmdStatusObject, pubCmdMoveActuator, pubDrop
    global pubCmdWorld, pubCmdDescribe, pubCmdTakeOrder, pubCmdExplain, pubCmdWhere, pubCmdDisp, pubCmdHappen

    rospy.init_node('planning_open_challenge')
    rospy.Subscriber("/planning_open_challenge/command_response", PlanningCmdClips, callbackCommandResponse)

    pubCmdSpeech = rospy.Publisher('/planning_open_challenge/cmd_speech', PlanningCmdClips, queue_size=1)
    pubCmdInt = rospy.Publisher('/planning_open_challenge/cmd_int', PlanningCmdClips, queue_size=1)
    pubCmdConf = rospy.Publisher('/planning_open_challenge/cmd_conf', PlanningCmdClips, queue_size=1)
    pubCmdGetTask = rospy.Publisher('/planning_open_challenge/cmd_task', PlanningCmdClips, queue_size=1)
    pubCmdGoto = rospy.Publisher('/planning_open_challenge/cmd_goto', PlanningCmdClips, queue_size=1)
    pubCmdAnswer = rospy.Publisher('/planning_open_challenge/cmd_answer', PlanningCmdClips, queue_size=1)
    pubCmdFindObject = rospy.Publisher('/planning_open_challenge/cmd_find_object', PlanningCmdClips, queue_size=1)
    pubCmdAskFor = rospy.Publisher('/planning_open_challenge/cmd_ask_for', PlanningCmdClips, queue_size=1)
    pubCmdStatusObject = rospy.Publisher('/planning_open_challenge/cmd_status_object', PlanningCmdClips, queue_size=1)
    pubUnknown = rospy.Publisher('/planning_open_challenge/cmd_unknown', PlanningCmdClips, queue_size=1)
    pubCmdMoveActuator = rospy.Publisher('/planning_open_challenge/cmd_move_actuator', PlanningCmdClips, queue_size=1)
    pubDrop = rospy.Publisher('/planning_open_challenge/cmd_drop', PlanningCmdClips, queue_size=1)

    pubCmdWorld = rospy.Publisher('/planning_open_challenge/cmd_world', PlanningCmdClips, queue_size=1)
    pubCmdDescribe = rospy.Publisher('/planning_open_challenge/cmd_describe', PlanningCmdClips, queue_size=1)
    pubCmdTakeOrder = rospy.Publisher('/planning_open_challenge/cmd_order', PlanningCmdClips, queue_size=1)
    pubCmdExplain = rospy.Publisher('/planning_open_challenge/cmd_explain', PlanningCmdClips, queue_size=1)
    pubCmdWhere = rospy.Publisher('/planning_open_challenge/cmd_where', PlanningCmdClips, queue_size=1)

    pubCmdDisp = rospy.Publisher('/planning_open_challenge/cmd_disp', PlanningCmdClips, queue_size=1)
    pubCmdHappen = rospy.Publisher('/planning_open_challenge/cmd_happen', PlanningCmdClips, queue_size=1)

    Initialize()
    
    tk.mainloop()

if __name__ == "__main__":
    main()
