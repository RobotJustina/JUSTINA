import serial, time

class Registers():
    ID = 3
    BAUD_RATE = 4
    RETURN_DELAY_TIME = 5
    CW_ANGLE_LIMIT = 6
    CCW_ANGLE_LIMIT = 8
    HIGHEST_LIMIT_TEMP = 11
    LOWEST_LIMIT_VOLT = 12
    HIGHEST_LIMIT_VOLT = 13
    MAX_TORQUE = 14

class ServoConstants():
    StatusReturnLevel = 2
    ModelAx_12 = 0
    ModelRX_64 = 1
    ModelRx_28 = 2
    ModelEx_106 = 3
    ModelMx_64 = 4
    ModelMx_106 = 5

#
#Initializes the serial port
#
def Open(comport, rate):
    global port
    port = serial.Serial(comport, rate, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout = 0.1);
    return
def Close():
    port.close()

#
#Methods for writing and reading bytes
#

def _write_byte(Id, address, value): #value should be an 8-bit data
    data = bytearray([255, 255, Id, 4, 3, address, value, 0])
    data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF)
    port.write(data)

def _write_word(Id, address, value): #Value should be a 16-bit data
    valueL = value & 0xFF
    valueH = (value >> 8) & 0xFF
    data = bytearray([255, 255, Id, 5, 3, address, valueL, valueH, 0])
    data[8] = ~((data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) & 0xFF)
    port.write(data)

def _read_byte(Id, address): #reads the 8-bit data stored in address
    data = bytearray([255, 255, Id, 4, 2, address, 1, 0])
    data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF)
    port.write(data)
    respStr = port.read(7) #When reading a byte, a 7-byte packet is expected: [255, 255, Id, lenght, error, value, checksum]
    if len(respStr) != 7:
        print "Dynamixel.->Error while reading address=" + str(address) + " id=" + str(Id) + ": received packet must have 7 bytes :'("
        return 0
    respBytes = bytearray(respStr)
    return respBytes[5]

def _read_word(Id, address): #reads the 16-bit data stored in address and address+1
    data = bytearray([255, 255, Id, 4, 2, address, 2, 0])
    data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF)
    port.write(data)
    respStr = port.read(8) #When reading a word, 8 bytes are expected: [255, 255, Id, lenght, error, valueL, valueH, checksum]
    if len(respStr) != 8:
        print "Dynamixel.->Error while reading address=" + str(address) + " id=" + str(Id) + ": received packet must have 8 bytes :'("
        return 0
    respBytes = bytearray(respStr)
    return ((respBytes[6] << 8) + respBytes[5])

#Each servo has a status return level. Here it's assumed that all servos will have the same status-return-level
#This function, with no arguments, returns the StatusReturnLevel that is suposes to be set to all servos.
#A similar function, but with an ID as an argument, returns the StatusReturnLevel of the servo with the ID
def GetStatusReturnLevel():
    return ServoConstants.StatusReturnLevel

def SetStatusReturnLevel(level):
    ServoConstants.StatusReturnLevel = level

def GetBaudrateBits(id, baudrate):

def GetStatusReturnLevel(id):
    return 0

def SetStatusReturnLevel(id, value):
    return 0
    
