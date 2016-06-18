'''
@author: arcra
'''

import threading, re
from abc import ABCMeta

class MessageTypes(object):
    __metaclass__ = ABCMeta
    COMMAND = 1
    RESPONSE = 2
    SHARED_VAR = 3

class Message(object):
    __metaclass__ = ABCMeta

    def __init__(self, commandName, params = None):
        self.name = commandName
        if params:
            self.params = params
        else:
            self.params = ''
        self._id = -1
        self.type = MessageTypes.RESPONSE
        self.isNotification = False
    
    def __eq__(self, other):
        return self.name == other.name and self._id == other._id
    
    def __hash__(self):
        return hash(self.name + str(self._id))
    
    def _isStandardCommand(self):
        return self.name in set(['busy',
                             'alive',
                             'ready'])
    def __repr__(self):
        textrep = self.name
        if not self._isStandardCommand():
            textrep += ' "' + str(self.params) + '"'
        if self.type in [MessageTypes.RESPONSE, MessageTypes.SHARED_VAR]:
            textrep += ' ' + str(int(self.successful))
        if self._id > -1:
            textrep += ' @' + str(self._id)
        return textrep

class Command(Message):
    
    _idCounter = 1
    _idLock = threading.Lock()
    
    __rx = re.compile(r'^((?P<src>[A-Za-z][A-Za-z\-]*)\s+)?(?P<cmd>[A-Za-z_]+)(\s+"(?P<params>(\\.|[^"])*)")?(\s+@(?P<id>\d+))?$')
    
    def __init__(self, commandName, params = "", idNum = None):
        '''
        Creates a command object.
        
        NOTICE: idNum parameter should be left alone, it's intended for internal use only. 
        '''
        super(Command, self).__init__(commandName, params)
        self.type = MessageTypes.COMMAND
        #Workaround for BB not returning ID on these commands:
        if commandName == 'write_var':
            return
        if idNum != None:
            self._id = idNum
        else:
            Command._idLock.acquire()
            self._id = Command._idCounter
            Command._idCounter += 1
            Command._idLock.release()
    
    @classmethod
    def Parse(cls, s):
        m = Command.__rx.match(s)
        if not m:
            return None
        
        sCommand = m.group('cmd').lower()
        sParams = m.group('params')
        sId = m.group('id')
        idNum = -1
        if sId and len(sId) > 0:
            idNum = int(sId)
        if sParams:
            sParams = sParams.replace("\\\"", "\"")
        return Command(sCommand, sParams, idNum)

class Response(Message):
    
    __rx = re.compile(r'^((?P<src>[A-Za-z][A-Za-z\-]*)\s+)?(?P<cmd>[A-Za-z_]+)(\s+"(?P<params>(\\.|[^"])*)")?\s+(?P<result>[10])(\s+@(?P<id>\d+))?$')
    
    def __init__(self, commandName, successful = False, response = ''):
        super(Response, self).__init__(commandName, response)
        self.type = MessageTypes.RESPONSE
        self.successful = successful
    
    @classmethod
    def Parse(cls, s):
        m = Response.__rx.match(s)
        if not m:
            return None
        
        sCommand = m.group('cmd').lower()
        sParams = m.group('params').strip()
        sId = m.group('id')
        sResult = m.group('result')
        idNum = -1
        
        if sResult is None or (sResult != '1' and sResult != '0'):
            return None
        if sId and len(sId) > 0:
            idNum = int(sId)
                    
        successful = int(sResult == '1')
        if sParams:
            sParams = sParams.replace("\\\"", "\"")
        
        r = Response(sCommand, successful, sParams)
        r._id = idNum
        return r
    
    @classmethod
    def FromCommandObject(cls, commandObj, successful = False, response = None):
        
        if not response:
            response = commandObj.params
        
        if not isinstance(commandObj, Command):
            raise Exception('commandObj not instance of Command in Response.FromCommandObject.')
        
        r = cls(commandObj.name, successful, response)
        r._id = commandObj._id
        return r