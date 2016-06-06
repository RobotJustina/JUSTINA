'''
@author: arcra
'''
import threading, BB

class ParallelSender(object):
    '''
        
    '''

    def __init__(self, command, timeout = 300, attempts = 1):
        '''
        Sends a command and wait for the answer in parallel to other thread's execution,
        allowing other thread's to poll if the response have been recevied.
        
        Params:
        command - Command to be sent, must be an instance of class Command.
        timeout - (Default 300) How much time (in seconds) to wait for response before trying again or aborting.
        attempts - (Default 1) How many attempts to send the command if no response is received after timeout.
                    If attempts is 0, it will keep trying indefinitely until StopSending is called.
        '''
        self.__sendingLock = threading.Lock()
        self.__sending = True
        
        self.__respLock = threading.Lock()
        self.__response = None
    
        self.__command = command
        
        self.__attemptsLock = threading.Lock()
        self.__attempts = attempts
        
        self.__timeout = timeout
        
        self.__p = threading.Thread(target=self.__Execute)
        self.__p.daemon = True
        self.__p.start()
        
    @property
    def sending(self):
        self.__sendingLock.acquire()
        r = self.__sending
        self.__sendingLock.release()
        return r
    
    def __setSending(self, s):
        self.__sendingLock.acquire()
        self.__sending = s
        self.__sendingLock.release()
    
    @property
    def response(self):
        if not self.__respLock.acquire(False):
            return None
        r = self.__response
        self.__respLock.release()
        return r
    
    def __setResponse(self, R):
        self.__respLock.acquire()
        self.__response = R
        self.__respLock.release()
    
    def StopSending(self):
        self.__attemptsLock.acquire()
        self.__attempts = 1
        self.__attemptsLock.release()
        
    def __Execute(self):
        
        response = None
        
        currentAttempt = 0
        
        self.__attemptsLock.acquire()
        att = self.__attempts
        self.__attemptsLock.release()
        
        while not response and (att == 0 or currentAttempt < att):
            currentAttempt += 1
            response = BB.SendAndWait(self.__command, self.__timeout)
            
            self.__attemptsLock.acquire()
            att = self.__attempts
            self.__attemptsLock.release()
        
        self.__setResponse(response)
        self.__setSending(False)
        