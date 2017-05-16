import socket
import threading
import sys

import BB

class ConnectionManager(object):
    '''
    Manages socket listening (waiting for blackboard or other client to connect),
    as well as sending messages and asynchronous receiving. It sends and receives
    through different socket objects to avoid problems related to multiprocessing.
    '''
    def __init__(self, port):
        '''
        Instanciates the ConnectionManager class.
        
        Params:
        port - Is the port to which the socket will be bound to listen for incoming connections.
        '''
        self.port = port
        self._clientIsConnected = False
        self._cicLock = threading.Lock()
        
        self._buildListenintSocket()
        
        self._connEstablishedCondition = threading.Event()
        
        self.listeningThread = threading.Thread(target=self._receivingThread)
        self.listeningThread.daemon = True
        
    def _buildListenintSocket(self):
        self.listeningSock = socket.socket()
        self.listeningSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listeningSock.bind(('0.0.0.0', self.port))
        self.listeningSock.listen(5)

    def Start(self):
        
        print 'Waiting for BlackBoard connection...'
        
        try:
            self.sock, self.remoteAddress = self.listeningSock.accept()
        except (socket.error, KeyboardInterrupt, SystemExit):
            self.listeningSock.close()
            sys.exit()
        
        self.clientIsConnected = True
        print 'Blackboard connected from {0}'.format(self.remoteAddress)
        
        self._connEstablishedCondition.clear()
        self.listeningThread.start()
        
        self._connEstablishedCondition.wait()
        
        self.listeningSock.close()
        
    def _accept(self):
        
        print 'Waiting for BlackBoard connection...'
        
        try:
            self.sock, self.remoteAddress = self.listeningSock.accept()
        except (socket.error, KeyboardInterrupt, SystemExit):
            self.listeningSock.close()
            sys.exit()
        
        self.clientIsConnected = True
        print 'Blackboard connected from {0}'.format(self.remoteAddress)
        
        self.listeningSock.close()
        
    @property
    def clientIsConnected(self):
        self._cicLock.acquire()
        r = self._clientIsConnected
        self._cicLock.release()
        return r
    
    @clientIsConnected.setter
    def clientIsConnected(self, val):
        self._cicLock.acquire()
        self._clientIsConnected = val
        self._cicLock.release()
    
    def _receivingThread(self):
        
        self._connEstablishedCondition.set()
        
        while True:
            if not self.clientIsConnected:
                self._buildListenintSocket()
                self._accept()

            try:
                data = self.sock.recv(4096)
            except:
                self.sock.close()
                data = ''
            if data == '':
                print 'BlackBoard disconnected.'
                self.clientIsConnected = False
                continue
            
            #print 'rcvd: ' + data
            while data:
                try:
                    pos = data.find('\0')
                    if pos == -1:
                        pos = len(data)
                    item = data[:pos].strip()
                    BB._incomingMessages.put(item, True, 5)
                    data = data[pos+1:]
                except:
                    print 'Could not enqueue message: ' + data[:pos]
    
    def Send(self, message):
        
        if not self.clientIsConnected:
            print 'Unable to send message, blackboard not connected'
            return None
        
        try:
            #print 'sending: ' + repr(message)
            sent = self.sock.send(repr(message) + '\0')
        except:
            print 'Something went bad while sending a message.'
            print '[Message:] ' + str(message)
            print type(message)
            return False
        
        if sent != len(repr(message)) + 1:
            print 'Message was sent incomplete.'
            return False
        
        return True
