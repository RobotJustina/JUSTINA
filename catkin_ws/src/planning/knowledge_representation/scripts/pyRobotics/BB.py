'''
@author: arcra
'''

import threading, time, types, Queue
import SharedVariables, ParallelSenders
from Messages import Message, Command, Response
from ConnectionManager import ConnectionManager
from CommandParser import CommandParser

ParallelSender = ParallelSenders.ParallelSender

SharedVarTypes = SharedVariables.SharedVarTypes
SubscriptionTypes = SharedVariables.SubscriptionTypes
ReportTypes = SharedVariables.ReportTypes

__initialized = False

__started = False
__startedLock = threading.Lock()

__ready = False
__readyLock = threading.Lock()

_subscriptionHandlersLock = threading.Lock()
_subscriptionHandlers = {}

_incomingMessages = Queue.Queue(20)
_receivedCommands = Queue.Queue(20)

_receivedResponses = {}
_responsesLock = threading.Lock()

_sentCommands = set([])
_commandsLock = threading.Lock()

def Initialize(port, functionMap={}, asyncHandler = None):
    global __executors, __connMan, __parser, __p, __initialized, __ready
    
    __executors = { 'busy' : (lambda x: Response('busy'), False),
                      'ready' : (__isReady, False),
                      'alive' : (lambda x: Response('alive', True), False) }

    for m in functionMap:
        if isinstance(functionMap[m], types.FunctionType):
            __executors[m] = (functionMap[m], False)
        elif isinstance(functionMap[m], tuple):
            __executors[m] = functionMap[m]
        else:
            print 'Element in function map is not a function nor a tuple: ' + repr(functionMap[m])
    
    __connMan = ConnectionManager(port)
    __parser = CommandParser(asyncHandler)
    
    __p = threading.Thread(target=__MainThread)
    __p.daemon = True
    
    __initialized = True

def Start():
    global __p, __connMann, __parser, __initialized, __started, __startedLock
    
    if not __initialized:
        print 'pyRobotics need to be initialized before starting.'
        return
    
    __parser.Start()
    __connMan.Start()
    __p.start()
    
    __startedLock.acquire()
    __started = True
    __startedLock.release()

def SetReady(val=True):
    global __ready, __readyLock
    
    __readyLock.acquire()
    __ready = val
    __readyLock.release()

def __isReady(c):
    global __ready, __readyLock
    
    __readyLock.acquire()
    ready = __ready
    __readyLock.release()
    
    return Response('ready', ready)

def Wait():
    while True:
        time.sleep(300)

def __MainThread():
    global _receivedCommands, __executors
    while True:
        command = _receivedCommands.get()
        key = command.name
        if key not in __executors:
            if '*' in __executors:
                key = '*'
            else:
                print 'Executor not found for command: ' + command.name
                return
        
        func, async = __executors[key]
        if async:
            p = threading.Thread(target=__Execute, args=(func, command))
            p.daemon = True
            p.start()
        else:
            __Execute(func, command)

def __Execute(func, command):
    try:
        response = func(command)
    except:
        print "Function '" + str(func) + "' crashed."
        response = Response.FromCommandObject(command, False, command.params)
    
    if not isinstance(response, Response):
        print "Function '" + str(func) + "' did not return a Response object."
    
    resp = Response.FromCommandObject(command, response.successful, response.params)
    
    Send(resp)

def Send(message):
    global __connMan, __started, __startedLock
    '''
    Sends a command WITHOUT waiting for an answer.
    
    Params:
    message - Message to be sent, must be an instance of a derived class of Message.
    '''
    
    __startedLock.acquire()
    _started = __started
    __startedLock.release()
    
    if not _started:
        print 'pyRobotics has not been started.'
        return False
    
    if not isinstance(message, Message):
        print "Message to be sent should be a derived class of pyRobotics.Messages.Message Class. Message was not sent."
        return False
    
    for _ in range(3):
        if __connMan.Send(message):
            return True
    
    return False

def SendAndWait(command, timeout=300000, attempts = 1):
    global _commandsLock, _sentCommands, _responsesLock, _receivedResponses, __started, __startedLock
    '''
    Sends a command and wait for the answer. This blocks the execution of the calling thread.
    
    Params:
    command - Command to be sent, must be an instance of class Command.
    timeout - (Default 300) How much time (in seconds) to wait for response before trying again or aborting.
    attempts - (Default 1) How many attempts to send the command if no response is received after timeout.
                If attempts is 0, it will keep trying indefinitely. (Not recommended)
    '''
    
    __startedLock.acquire()
    _started = __started
    __startedLock.release()
    
    if not _started:
        print 'pyRobotics has not been started.'
        return None
    
    if not isinstance(command, Command):
        print "Message should be an instance of class Command. Message not sent."
        return None
    
    _commandsLock.acquire()
    _sentCommands.add(command)
    _commandsLock.release()
    
    currentAttempt = 0
    
    timeout = timeout/1000.0
    
    response = None
    
    while not response and currentAttempt < attempts:
        Send(command)
        newTimeout = time.time() + timeout
        currentAttempt += 1
        while time.time() < newTimeout:
            _responsesLock.acquire()
            if command in _receivedResponses:
                response = _receivedResponses.pop(command)
            _responsesLock.release()
            if response:
                break
            time.sleep(0.3)
    
    _commandsLock.acquire()
    _sentCommands.remove(command)
    _commandsLock.release()
    
    return response

def ReadSharedVar(name):
    global __started, __startedLock
    
    __startedLock.acquire()
    _started = __started
    __startedLock.release()
    
    if not _started:
        print 'pyRobotics has not been started.'
        return False
    
    return SharedVariables._ReadSharedVar(name)

def CreateSharedVar(sharedVarType, name):
    global __started, __startedLock
    
    __startedLock.acquire()
    _started = __started
    __startedLock.release()
    
    if not _started:
        print 'pyRobotics has not been started.'
        return False
    
    return SharedVariables._CreateSharedVar(sharedVarType, name)

def WriteSharedVar(sharedVarType, name, data):
    global __started, __startedLock
    
    __startedLock.acquire()
    _started = __started
    __startedLock.release()
    
    if not _started:
        print 'pyRobotics has not been started.'
        return False
    
    return SharedVariables._WriteSharedVar(sharedVarType, name, data)

def SubscribeToSharedVar(name, handler, subscriptionType=SharedVariables.SubscriptionTypes.WRITE_OTHERS, reportType = SharedVariables.ReportTypes.CONTENT):
    global _subscriptionHandlersLock, _subscriptionHandlers, __started, __startedLock
    
    __startedLock.acquire()
    _started = __started
    __startedLock.release()
    
    if not _started:
        print 'pyRobotics has not been started.'
        return False
    
    if not SharedVariables._SubscribeToSharedVar(name, subscriptionType, reportType):
        return False
    
    _subscriptionHandlersLock.acquire()
    _subscriptionHandlers[name] = handler
    _subscriptionHandlersLock.release()
    
    return True
