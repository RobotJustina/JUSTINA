# -*- coding: utf-8 -*-
'''
@author: arcra
'''
import time, threading, os
import Tkinter as tk

import clipsFunctions
from clipsFunctions import clips

import pyRobotics.BB as BB
from pyRobotics.Messages import Command, Response

#from BBFunctions import gui
from BBFunctions import ResponseReceived, CreateSharedVar, WriteSharedVar, SubscribeToSharedVar, RunCommand

defaultTimeout = 2000
defaultAttempts = 1


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
    cmd = Command(cmdName, params)
    BB.Send(cmd)
    return cmd._id

def SendResponse(cmdName, cmd_id, result, response):
    result = str(result).lower() not in ['false', '0']
    r = Response(cmdName, result, response)
    r._id = cmd_id
    BB.Send(r)

def Initialize():
    clips.Memory.Conserve = True
    clips.Memory.EnvironmentErrorsEnabled = True
    
    clips.RegisterPythonFunction(SendCommand)
    clips.RegisterPythonFunction(SendResponse)
    clips.RegisterPythonFunction(setCmdTimer)
    clips.RegisterPythonFunction(setTimer)
    clips.RegisterPythonFunction(CreateSharedVar)
    clips.RegisterPythonFunction(WriteSharedVar)
    clips.RegisterPythonFunction(SubscribeToSharedVar)
    
    clips.BuildGlobal('defaultTimeout', defaultTimeout)
    clips.BuildGlobal('defaultAttempts', defaultAttempts)
    
    filePath = os.path.dirname(os.path.abspath(__file__))
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'BB_interface.clp')
    
    BB.Initialize(2000, functionMap = {'*':RunCommand}, asyncHandler = ResponseReceived)
    
    BB.Start()
    

def main():
    Initialize()
    
    tk.mainloop()

if __name__ == "__main__":
    main()