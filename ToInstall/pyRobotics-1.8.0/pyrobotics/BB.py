#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This module contains the main interface to interact with BlackBoard.

Author: Adrián Revuelta Cuauhtli <adrianrc.89@gmail.com>

Workplace: Bio-Robotics Lab., UNAM <http://bio-robotics.fi-p-unam.mx>
'''
# STANDARD IMPORTS
import threading
import time
import types
import Queue
import sys

# PACKAGE IMPORTS
import shared_variables, parallel_senders
from messages import Message, Command, Response
from connection_manager import ConnectionManager
from command_parser import CommandParser

__version__ = '1.8.0'

ParallelSender = parallel_senders.ParallelSender

SharedVarTypes = shared_variables.SharedVarTypes
SubscriptionTypes = shared_variables.SubscriptionTypes
ReportTypes = shared_variables.ReportTypes

_initialized = False

_started = False
_startedLock = threading.Lock()

_ready = False
_readyLock = threading.Lock()

_subscriptionHandlersLock = threading.Lock()
_subscriptionHandlers = {}

_incomingMessages = Queue.Queue(20)
_receivedCommands = Queue.Queue(20)

_receivedResponses = {}
_responsesLock = threading.Lock()

_sentCommands = set([])
_commandsLock = threading.Lock()

def Initialize(port, functionMap={}, asyncHandler = None):
    '''
    
    Initializes BlackBoard with the corresponding parameters.
    
    :param int port: The port through which BlackBoard will communicate with this module.
    :param dictionary functionMap: A dictionary containing **key:value** pairs, where the *key* is the name of a command received (a string),
        and the *value* is either a tuple containing a function as a first element and a boolean as a second element, or a function.
        The function in both cases is the function that is going to execute the specified command and receives on object of type :class:`Command` (See :ref:`Creating a command handler <creating_a_command_handler>`).
        The boolean value indicates whether the execution of that command should be synchronous (on the same thread) or asynchronous,
        usually synchronous execution is preferred for fast commands that can answer almost immediately and asynchronous for commands that might take a little time.
        When the value is only a function, by default the execution is synchronous. *functionMap* can also contain an entry with a string containing only an asterisk,
        meaning that would be the handler in case no other handler is found for a specific command.
        
        .. note::

            Notice that although functionMap can include a wildcard handler and this might seem like the module could answer
            anything, BlackBoard will only send commands that are registered under this module's configuration.
        
    :param function asyncHandler: A function that would handle the response of commands when sent with the method :func:`Send`
        instead of using :func:`SendAndWait`. This means the execution of a program that sends a command could continue
        and an asynchronous handler would handle the response when one is received.

        .. note::
    
            Notice that the asyncHandler functionality could also be achieved using a :class:`ParallelSender` object,
            but it has other implications.
    
    '''
    global _executors, _connMan, _parser, _p, _initialized, _ready
    
    _executors = { 'busy' : (lambda x: Response('busy'), False),
                      'ready' : (_isReady, False),
                      'alive' : (lambda x: Response('alive', True), False) }

    for m in functionMap:
        if isinstance(functionMap[m], types.FunctionType):
            _executors[m] = (functionMap[m], False)
        elif isinstance(functionMap[m], tuple):
            _executors[m] = functionMap[m]
        else:
            print 'Element in function map is not a function nor a correct tuple: ' + repr(functionMap[m])
    
    _connMan = ConnectionManager(port)
    _parser = CommandParser(asyncHandler)
    
    _p = threading.Thread(target=_MainThread)
    _p.daemon = True
    
    _initialized = True

def Start():
    '''
    Once pyRobotics is :func:`initialized <Initialize>`, you can start the communication with BlackBoard.
    This will start the threads of the internal *ConnectionManager* and *CommandParser* classes to start listening for
    a connection and start receiving and parsin messages.
    
    If pyRobotics is not initialized it will only print a message saying "pyRobotics needs to be initialized before starting".
    A similar message will show when trying to use some of this module's functions before calling this function.
    
    .. todo::
    
        Fix bug: sometimes when connection is established successfully a message saying pyRobotics has not been started is printed.
    
    '''
    global _p, _connMann, _parser, _initialized, _started, _startedLock
    
    if not _initialized:
        print 'pyRobotics needs to be initialized before starting.'
        return
    
    _parser.Start()
    _connMan.Start()
    _p.start()
    
    _startedLock.acquire()
    _started = True
    _startedLock.release()

def SetReady(val=True):
    '''
    Once pyRobotics is :func:`initialized <Initialize>` and :func:`started <Start>`, this flag should be set to true to
    let BlackBoard know that the module is functioning correctly and ready to receive commands.
    Even if this module does not receive any commands, this should be set to true.
    '''
    global _ready, _readyLock, _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    _readyLock.acquire()
    _ready = val
    _readyLock.release()

def _isReady(c):
    global _ready, _readyLock
    
    _readyLock.acquire()
    ready = _ready
    _readyLock.release()
    
    return Response('ready', ready)

def Wait():
    '''
    In case this module is only used to receive and respond commands, but is doing nothing while no command is received,
    this will prevent the main thread (and therefore BlackBoard connection and commands execution) to terminate.
    '''
    global _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    try:
        while True:
            time.sleep(300)
    except (KeyboardInterrupt, SystemExit):
        sys.exit()

def _MainThread():
    global _receivedCommands, _executors
    while True:
        command = _receivedCommands.get()
        key = command.name
        if key not in _executors:
            if '*' in _executors:
                key = '*'
            else:
                print 'Executor not found for command: ' + command.name
                return
        
        func, async = _executors[key]
        if async:
            p = threading.Thread(target=_Execute, args=(func, command))
            p.daemon = True
            p.start()
        else:
            _Execute(func, command)

def _Execute(func, command):
    try:
        response = func(command)
    except Exception as e:
        print "Function '" + str(func) + "' crashed."
        print 'ERROR: ' + str(e)
        response = Response.FromCommandObject(command, False, command.params)
    
    if not isinstance(response, Response):
        print "Function '" + str(func) + "' did not return a Response object."
        response = Response.FromCommandObject(command, False, command.params)
    
    resp = Response.FromCommandObject(command, response.successful, response.params)
    
    Send(resp)

def Send(message):
    '''
    Sends a command WITHOUT waiting for an answer.
    
    :param Command message: Message to be sent, must be an instance of the Command class.
    :return: ``True`` if the message was sent successfully, ``False`` otherwise.
    '''
    global _connMan, _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    if not isinstance(message, Message):
        print "Message to be sent should be a derived class of pyrobotics.messages.Message Class. Message was not sent."
        return False
    
    for _ in range(3):
        if _connMan.Send(message):
            return True
    
    return False

def SendAndWait(command, timeout=300000, attempts = 1):
    global _commandsLock, _sentCommands, _responsesLock, _receivedResponses, _started, _startedLock
    '''
    Sends a command and wait for the answer. This blocks the execution of the calling thread.
    
    :param Command command: Message to be sent, must be an instance of the Command class.
    :param int timeout: (Default 300000) How much time (in miliseconds) to wait for response before trying again or aborting.
    :param int attempts: (Default 1) How many attempts to send the command if no response is received after timeout.
    If attempts is 0, it will keep trying indefinitely. (Not recommended)
    :return: A :class:`Response` object if the message was sent successfully and a response was received before the timeout occurred, ``None`` otherwise.
    '''
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
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
            time.sleep(0.1)
    
    _commandsLock.acquire()
    _sentCommands.remove(command)
    _commandsLock.release()
    
    return response

def ReadSharedVar(name):
    '''
    Reads the value of a Shared Variable from the BlackBoard.
    
    :param string name: The name of the Shared Variable.
    :return: A :class:`SharedVar` object if the request was successful, ``False`` if pyRobotics has not been started, ``None`` otherwise.
    '''
    global _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    return shared_variables._ReadSharedVar(name)

def CreateSharedVar(sharedVarType, name):
    '''
    Creates a Shared Variable in BlackBoard.
    
    :param enum sharedVarType: The type of the shared variable, it is one of the constants in :class:`SharedVarTypes` pseudo-enum.
    :param string name: The name of the shared variable to be created.
    :return: ``True`` if creation was successful, ``False`` otherwise.
    '''
    global _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    return shared_variables._CreateSharedVar(sharedVarType, name)

def WriteSharedVar(sharedVarType, name, data):
    '''
    Writes content to a Shared Variable in BlackBoard.
    
    :param enum sharedVarType: The type of the shared variable, it is one of the constants in :class:`SharedVarTypes` pseudo-enum.
    :param string name: The name of the shared variable to write to.
    :param var data: The data to be written, the type must match the shared variable's type.
    :return: ``True`` if shared variable was succesfully written to, ``False`` otherwise.
    '''
    global _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    return shared_variables._WriteSharedVar(sharedVarType, name, data)

def SubscribeToSharedVar(name, handler, subscriptionType=SubscriptionTypes.WRITE_OTHERS, reportType = ReportTypes.CONTENT):
    '''
    Subscribes to a Shared Variable in BlackBoard.
    When a module subscribes to a shared variable, it gets notifications when someone writes to it.
    
    :param string name: The name of the shared variable to subscribe to.
    :param function handler: A function that will be the handler for this shared variables notification. (See :ref:`Creating a subscription handler <creating_a_subscription_handler>`)
    :param enum subscriptionType: The type of subscription, it is one of the constants in :class:`SubscriptionTypes` pseudo-enum.
    :param enum reportType: The type of report to receive when someone writes to it, it is one of the constants in :class:`ReportTypes` pseudo-enum.
    :return: ``True`` if subscription was successful, ``False`` otherwise.
    '''
    global _subscriptionHandlersLock, _subscriptionHandlers, _started, _startedLock
    
    _startedLock.acquire()
    started = _started
    _startedLock.release()
    
    if not started:
        print 'pyRobotics has not been started.'
        return False
    
    if not shared_variables._SubscribeToSharedVar(name, subscriptionType, reportType):
        return False
    
    _subscriptionHandlersLock.acquire()
    _subscriptionHandlers[name] = handler
    _subscriptionHandlersLock.release()
    
    return True
