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

class ServoProp():
    StatusReturnLevel = 2
    ModelAx_12 = 0
    ModelRX_64 = 1
    ModelRx_28 = 2
    ModelEx_106 = 3
    ModelMx_64 = 4
    ModelMx_106 = 5

#Initializes the serial port
def Open(comport, rate):
    global port
    port = serial.Serial(comport, rate, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout = 0.1);
    return
def Close():
    port.close()
    
