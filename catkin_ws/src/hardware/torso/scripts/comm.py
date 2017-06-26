#!/usr/bin/env python
import serial
import errno
import struct

# This module will control communication beween PC and Arduino board

#CONST
BOARD_PORT = "/dev/justinaTorso"
SPEED      = 115200
TIMEOUT    = 1
SYNC       = 254                #byte de comienzo de los mensajes seriales
ESCAPE     = 253                #byte de desambiguacion en los mensajes seriales
BROADCAST  = 255   


#device ID
PC_ID      = 0
ARDUINO_ID = 1

#mod list
MOD_SYSTEM  = 0
MOD_SENSORS = 1
MOD_MOTORS  = 2

#system opcodes 
OP_PING      = 0              
OP_VERSION   = 1
OP_LISTMOD   = 2
OP_ERROR     = 3
OP_STOP      = 4

#sensors opcodes
OP_SETDISTMAX        = 0
OP_GETDISTMAX        = 1
OP_SETDISTMIN        = 2
OP_GETDISTMIN        = 3
OP_GETCURRENTDIST    = 4

#motors opcodes
OP_SETTORSOPOSE  = 0
OP_CALIBRATE     = 1
OP_GOUP          = 2
OP_GODOWN        = 3
OP_STOP_MOTOR    = 4 


#ESTADOS
BUSCAR, LEER_LARGO, LEER_MSG, CHECKSUM = range (0,4) #set automaticaly values to our state variables
CONNECTION_TRIES = 5
RECONNECT_SLEEP  = 2

class Comm():
    
    def __init__(self , board_port=BOARD_PORT):        
        self.estado = BUSCAR
        self.byteData = 0
        self.largo = 0
        self.byteAnt = 0
        self.checksum = 0
        self.cont = 0
        self.comando = []   
        self.mensajesRecv = 0
        #init serial or usb communication
        try:
            self.board = serial.Serial(board_port, SPEED, timeout = TIMEOUT)
        except Exception, e:
            print "Comm: Error al intentar conectarse"
            #tratar de reconectarse
    
    #send data to mvdShield
    def send(self, msg):
        try:
            #armar el ensaje a enviar en un buffer
            self.comando = []                                   #limpio el buffer -- ver si no hay una manera piu pulita de borrar esa memoria (del o algo asi)
            self.comando.append(msg.iDe)
            self.comando.append(msg.mod)
            self.comando.append(msg.op)
            it = 0      

            #obtengo los parametros para mandar  
            while it < msg.largo:
                self.comando.append(msg.param[it])
                it = it + 1
           
            #escapear el mensaje
            pos = 0
            it = 0
            salida = []
            while it < msg.largo + 3:                           #le sumo tres para recorrer la data incluyendo al principio el id, mod y opcode
                if self.comando[it] != ESCAPE and self.comando[it] != SYNC:
                    salida.append(self.comando[it])
                else:
                    salida.append(ESCAPE)
                    salida.append(self.comando[it])
                    pos = pos + 1                
                pos = pos + 1
                it = it + 1
          
            #encapsula y envia el mensaje
            self.board.write(self.int2byte(SYNC, 8))
            self.board.write(self.int2byte(pos,8))
            self.checksum = 0
            it = 0
            while it < pos:
                self.board.write(self.int2byte(salida[it],8))
                self.checksum = self.checksum + salida[it]
                it = it + 1
            self.board.write(self.int2byte(self.checksum % 127,8))
        except:
            self.reconnect()


    def recv(self):    
        try:
            #recieve data from mvdShield
            while self.board.inWaiting() > 0:
                self.byteData = self.board.read()                                            #con el read() leo de a byte
                self.byteData = struct.unpack("B", self.byteData[0])[0]                          #convert a byte to a int
                #print "dato leido = " + str(self.byteData)

                if (self.byteData == SYNC and ( (self.estado == BUSCAR and self.byteAnt != ESCAPE ) or self.byteAnt < 128) ):
                    self.estado = LEER_LARGO                
                elif (self.estado == LEER_LARGO):
                    self.largo = self.byteData                                                 #largo no puede ser > 255 
                    self.checksum = 0                                                     #se prepara para leer el cuerpo del mensaje
                    self.cont = 0
                    self.comando = []                                                               #lista donde almaceno el mensaje nuevo
                    self.estado = LEER_MSG
                elif (self.estado == LEER_MSG):
                    self.comando.append(self.byteData)                                    #salvar dato en el buffer
                    self.cont = self.cont + 1
                    self.checksum = self.checksum + self.byteData
                    if self.cont == self.largo:
                        self.checksum = self.checksum % 128
                        self.estado = CHECKSUM
                elif (self.estado == CHECKSUM):
                    if self.checksum == self.byteData:
                        self.largo = self.unescape()                                                 #unescape largo
                        self.estado = BUSCAR
                        return self.parseMsg()   
                    else:
                        print "checksum bad"
                self.byteAnt = self.byteData
            return None                                                                 #en caso de que no haya nada para leer
        except:
            self.reconnect()
       
    def unescape(self):                                                              #saca los ESCAPES y devuelve el mensaje original
	#print "unescapeando"
        cont = 0
        pos = 0
        while cont < self.largo:
            #print "Comm: comando= " + str(self.comando[cont])
            if self.comando[cont] == ESCAPE:
                cont = cont + 1
            self.comando[pos] = self.comando[cont]
            pos = pos + 1
            cont = cont + 1
	#print "fin unescapeo"
        return pos

    def parseMsg(self):
        #print "Comm: en parseMsg"	
        msg = Msg()
        msg.setID(self.comando[0])
        msg.setModule(self.comando[1])
        msg.setOpcode(self.comando[2])
        msg.setLargo(self.largo)
        #print "Comm: largo = " + str(self.largo)
        cont = 0
        while cont < self.largo - 3:                                                #le resto 3 para descontar el ide, mod y opcode
            #print "Comm: parametro= " + str(self.comando[cont + 3])
            msg.addParam(self.comando[cont + 3])
            cont = cont + 1
        #print "Comm: saliendo de parseMsg"
        return msg        

    def leerRaw(self):
        dataWaiting = self.board.inWaiting()
        if dataWaiting > 0:
            print "Hay datos para leer: " + str(dataWaiting)
            data = self.board.read()
            data = struct.unpack("B", data[0])[0]  #convert a byte to a int
            print "dato = " + str(data)
            if data == 254:
                print "syn encontrado"
                print str(11%5)

    def int2byte(self, val, width=32):
        """
        Convert a signed integer value of specified width into a byte string.
        """
        if val < 0:
            val = val + (1 << width)
        return ''.join([chr((val >> 8*n) & 255) for n in reversed(range(width/8))]) 

    def reconnect(self, board_port=BOARD_PORT):
        connected = False
        tries = 0
        while not connected and tries < CONNECTION_TRIES:
            time.sleep(RECONNECT_SLEEP)
            try:
                "Trying to reconnect"
                self.board = serial.Serial(board_port, SPEED, timeout = TIMEOUT)
                connected = True
            except Exception, e:
                print "Comm::reconnet::Error al intentar conectarse"
                tries = tries + 1
            


class Msg():

    def __init__(self, iDe = None, mod = None, op = None, param = [], largo = None):
        self.iDe = iDe
        self.mod = mod
        self.op = op
        self.param = []
        if param != []:
            self.param.append(param)
        self.largo = largo

    def setID(self, iDe):
        self.iDe = iDe

    def setModule(self, mod):
        self.mod = mod

    def setOpcode(self, op):
        self.op = op

    def addParam(self, param):
        self.param.append(param)

    def setLargo(self, largo):                          #cantidad de parametros
        self.largo = largo

    def setMsg(self, iDe, mod, op, param, largo):   #this operation only add one parameter
        self.iDe = iDe
        self.mod = mod
        self.op = op
        self.param.append(param)
        self.largo = largo
