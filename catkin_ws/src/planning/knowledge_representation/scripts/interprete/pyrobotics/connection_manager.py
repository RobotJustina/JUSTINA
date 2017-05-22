import socket
import threading

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
        self.__clientIsConnected = False
        self.__cicLock = threading.Lock()
        
        self.__buildListenintSocket()
        
        self.__connEstablishedCondition = threading.Event()
        
        self.listeningThread = threading.Thread(target=self.__receivingThread)
        self.listeningThread.daemon = True
        
    def __buildListenintSocket(self):
        self.listeningSock = socket.socket()
        self.listeningSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listeningSock.bind(('0.0.0.0', self.port))
        self.listeningSock.listen(5)

    def Start(self):
        
        self.sock, self.remoteAddress = self.listeningSock.accept()
        
        self.clientIsConnected = True
        print 'Blackboard connected from {}'.format(self.remoteAddress)
        
        self.__connEstablishedCondition.clear()
        self.listeningThread.start()
        
        self.__connEstablishedCondition.wait()
        
        self.listeningSock.close()
        
    def __accept(self):
        
        self.sock, self.remoteAddress = self.listeningSock.accept()
        
        self.clientIsConnected = True
        print 'Blackboard connected from {}'.format(self.remoteAddress)
        
        self.listeningSock.close()
        
    @property
    def clientIsConnected(self):
        self.__cicLock.acquire()
        r = self.__clientIsConnected
        self.__cicLock.release()
        return r
    
    @clientIsConnected.setter
    def clientIsConnected(self, val):
        self.__cicLock.acquire()
        self.__clientIsConnected = val
        self.__cicLock.release()
    
    def __receivingThread(self):
        
        self.__connEstablishedCondition.set()
        
        while True:
            if not self.clientIsConnected:
                self.__buildListenintSocket()
                self.__accept()

            try:
                data = self.sock.recv(4096)
            except:
                self.sock.close()
                data = ''
            if data == '':
                print 'Client disconnected.'
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
                    print 'TEST_INCOMING_COMMAND'
                    print item
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
            
