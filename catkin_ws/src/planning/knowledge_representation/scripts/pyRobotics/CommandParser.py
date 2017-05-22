'''
@author: arcra
'''

import threading, BB
from Messages import Command, Response
from SharedVariables import SharedVar

class CommandParser(object):
        
    def __init__(self, asyncHandler = None):
        
        self.__p = threading.Thread(target=self.__parsingThread)
        self.__p.daemon = True
        
        self.__asyncHandler = asyncHandler 
    
    def Start(self):
        self.__p.start()
    
    def __parsingThread(self):
        
        while True:
            
            data = BB._incomingMessages.get()
            
            el = SharedVar.Parse(data)
            if el and el.isNotification:
                handler = None
                BB._subscriptionHandlersLock.acquire()
                if el.varName in BB._subscriptionHandlers:
                    handler = BB._subscriptionHandlers[el.varName]
                BB._subscriptionHandlersLock.release()
                
                if not handler:
                    print 'ERROR: No handler for shared variable: ' + el.varName
                    continue
                    
                try:
                    handler(el)
                except:
                    print 'Handler for shared var: "' + el.varName + '" crashed.'
                
                continue
            if not el:
                el = Response.Parse(data)
                if el:
                    print "Something that wasn't supposed to happen happened"
            if el:
                BB._commandsLock.acquire()
                if el in BB._sentCommands:
                    BB._responsesLock.acquire()
                    BB._receivedResponses[el] = el
                    BB._responsesLock.release()
                elif self.__asyncHandler:
                    self.__asyncHandler(el)
                else:
                    print 'Response without awaiting command: ' + repr(el)
                BB._commandsLock.release()
                continue
            el = Command.Parse(data)
            if el:
                BB._receivedCommands.put(el)
                continue
            
            print 'Invalid message received: ' + data + '_len(' + str(len(data)) + ')'
            
