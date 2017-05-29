import threading

import BB
from messages import Command, Response
from shared_variables import SharedVar

class CommandParser(object):
        
    def __init__(self, asyncHandler = None):
        
        self._p = threading.Thread(target=self._parsingThread)
        self._p.daemon = True
        
        self._asyncHandler = asyncHandler 
    
    def Start(self):
        self._p.start()
    
    def _parsingThread(self):
        
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
                elif self._asyncHandler:
                    self._asyncHandler(el)
                else:
                    print 'Response without awaiting command: ' + repr(el)
                BB._commandsLock.release()
                continue
            el = Command.Parse(data)
            if el:
                BB._receivedCommands.put(el)
                continue
            
            print 'Invalid message received: ' + data + '_len(' + str(len(data)) + ')'
            
