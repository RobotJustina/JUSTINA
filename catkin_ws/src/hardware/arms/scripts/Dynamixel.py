import serial, time

class Registers():
    MODEL_NUMBER = 0
    FIRMWARE_VERSION = 2
    ID = 3
    BAUD_RATE = 4
    RETURN_DELAY_TIME = 5
    CW_ANGLE_LIMIT = 6
    CCW_ANGLE_LIMIT = 8
    HIGHEST_LIMIT_TEMP = 11
    LOWEST_LIMIT_VOLT = 12
    HIGHEST_LIMIT_VOLT = 13
    MAX_TORQUE = 14
    STATUS_RETURN_LEVEL = 16
    ALARM_LED = 17
    ALARM_SHUTDOWN = 18
    TORQUE_ENABLE = 24
    LED = 25
    CW_COMPLIANCE_MARGIN = 26
    CCW_COMPLIANCE_MARGIN = 27
    CW_COMPLIANCE_SLOPE = 28
    CCW_COMPLIANCE_SLOPE = 29
    GOAL_POSITION = 30
    MOVING_SPEED = 32
    TORQUE_LIMIT = 34
    PRESENT_POSITION = 36
    PRESENT_SPEED = 38
    PRESENT_LOAD = 40
    PRESENT_VOLTAGE = 42
    PRESENT_TEMPERATURE = 43
    REGISTERED_INSTRUCTION = 44
    MOVING = 46
    LOCK = 47
    PUNCH = 48

class ServoConstants():
    ModelAx_12 = 0
    ModelRX_64 = 1
    ModelRx_28 = 2
    ModelEx_106 = 10
    ModelMx_64 = 11
    ModelMx_106 = 12
    

class DynamixelMan:
    'Class for communicating with a set of dynamixel servomotors connected to the same bus'
    def __init__(self, portName, baudrate):
        print "Open DynamixelMan in port " + portName + " at " + str(baudrate)
        print "Openning dynamixel on " + portName + " at " + str(baudrate)
        self.port = serial.Serial(portName, baudrate, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.1)
        self.StatusReturnLevel = 1

    def Close(self):
        self.port.Close()

    ###
    ###Methods for writing and reading bytes
    ###
    def _write_byte(self, Id, address, value): #value should be an 8-bit data
        data = bytearray([255, 255, Id, 4, 3, address, value, 0])
        data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF) & 0xFF
        self.port.write(data)

    def _write_word(self, Id, address, value): #Value should be a 16-bit data
        valueL = value & 0xFF
        valueH = (value >> 8) & 0xFF
        data = bytearray([255, 255, Id, 5, 3, address, valueL, valueH, 0])
        data[8] = ~((data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) & 0xFF) & 0xFF
        self.port.write(data)

    def _read_byte(self, Id, address): #reads the 8-bit data stored in address
        data = bytearray([255, 255, Id, 4, 2, address, 1, 0])
        data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF) & 0xFF 
        self.port.write(data)
        respStr = self.port.read(7) #When reading a byte, a 7-byte packet is expected: [255, 255, Id, lenght, error, value, checksum]
        if len(respStr) != 7:
            print "Dynamixel.->Error while reading addr=" + str(address) + " id=" + str(Id) + ": received packet must have 7 bytes :'("
            return 0
        respBytes = bytearray(respStr)
        return respBytes[5]

    def _read_word(self, Id, address): #reads the 16-bit data stored in address and address+1
        data = bytearray([255, 255, Id, 4, 2, address, 2, 0])
        data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF) & 0xFF
        #print "Sending: " + str(int(data[0])) + " " + str(int(data[1])) + " " + str(int(data[2])) + " " + str(int(data[3])) + " " + str(int(data[4])) + " " + str(int(data[5])) + " " + str(int(data[6])) + " " + str(int(data[7]))
        self.port.write(data)
        respStr = self.port.read(8) #When reading a word, 8 bytes are expected: [255, 255, Id, lenght, error, valueL, valueH, checksum]
        respBytes = bytearray(respStr)
        #print "Received: "
        #for i in range(len(respBytes)):
        #    print str(int(respBytes[i])) + " "
        if len(respStr) != 8:
            print "Dynamixel.->Error while reading addr=" + str(address) + " id=" + str(Id) + ": received packet must have 8 bytes :'("
            return 0
        return ((respBytes[6] << 8) + respBytes[5])

    #Each servo has a status return level, nevertheless, here it's assumed that all servos wired to the same bus will have the same status-return-level
    #This function, with no arguments, returns the StatusReturnLevel that is suposed to be set in all servos wired to the same bus
    #A similar function, but with an ID as an argument, returns the StatusReturnLevel of the servo with such ID
    def Ping(self, Id):
        data = bytearray([255, 255, Id, 2, 1, 0])
        data[5] = ~((data[2] + data[3] + data[4]) & 0xFF) & 0xFF
        print "Sending: " + str(int(data[0])) + " " + str(int(data[1])) + " " + str(int(data[2])) + " " + str(int(data[3])) + " " + str(int(data[4])) + " " + str(int(data[5]))
        self.port.write(data)
        respStr = self.port.read(6)
        respBytes = bytearray(respStr)
        print "Received: "
        for i in range(len(respBytes)):
            print str(int(respBytes[i])) + " "
        
    def GetStatusReturnLevel(self):
        return self.StatusReturnLevel

    def SetStatusReturnLevel(self, statusReturnLevel):
        self.StatusReturnLevel = statusReturnLevel

    ###
    ###Methods for reading and writing values to specific registers
    ###
    def GetId(self, Id):
        return self._read_byte(Id, Registers.ID)

    def SetId(self, Id, newId):
        self._write_byte(Id, Registers.ID, newId)

    #Returns baudrate in bps. Translation from bits to baudrate is made according to the formula given in the datasheet
    def GetBaudrate(self, Id):
        baudBits = self._read_byte(Id, Registers.BAUD_RATE)
        return int(2000000/(baudBits + 1)) #This formula is given in the datasheet

    def SetBaudrate(self, Id, baudrate):
        baudBits = int((2000000/baudrate) - 1)
        self._write_byte(Id, Registers.BAUD_RATE, baudBits)

    def GetCWAngleLimit(self, Id):
        return self._read_word(Id, Registers.CW_ANGLE_LIMIT)

    def SetCWAngleLimit(self, Id, angleLimit):
        self._write_word(Id, Registers.CW_ANGLE_LIMIT, angleLimit)

    def GetCCWAngleLimit(self, Id):
        return self._read_word(Id, Registers.CCW_ANGLE_LIMIT)

    def SetCCWAngleLimit(self, Id, angleLimit):
        self._write_word(Id, Registers.CCW_ANGLE_LIMIT, angleLimit)

    def GetStatusReturnLevel(self, Id):
        return self._read_byte(Id, Registers.STATUS_RETURN_LEVEL)

    def SetStatusReturnLevel(self, Id, stautusReturnLevel):
        self._write_byte(Id, Registers.STATUS_RETURN_LEVEL, stautusReturnLevel)

    def GetTorqueEnable(self, Id):
        return self._read_byte(Id, Registers.TORQUE_ENABLE)

    def SetTorqueEnable(self, Id, enable):
        self._write_byte(Id, Registers.TORQUE_ENABLE, enable)

    #Goal position could be in [0,1023] or [0,4095] depending on the servo model
    def GetGoalPosition(self, Id):
        return self._read_word(Id, Registers.GOAL_POSITION)

    def SetGoalPosition(self, Id, goalPose):
        self._write_word(Id, Registers.GOAL_POSITION, goalPose)

    def GetMovingSpeed(self, Id):
        return self._read_word(Id, Registers.MOVING_SPEED)

    def SetMovingSpeed(self, Id, movingSpeed):
        self._write_word(Id, Registers.MOVING_SPEED, movingSpeed)

    def SetTorqueVale(self, Id, torqueValue, directionTurn):
        if directionTurn == True:
            torqueValue = torqueValue + 1024
        self._write_word(Id, Registers.MOVING_SPEED, torqueValue)

    def GetTorqueLimit(self, Id):
        return self._read_word(Id, Registers.TORQUE_LIMIT)

    def SetTorqueLimit(self, Id, torqueLimit):
        self._write_word(Id, Registers.TORQUE_LIMIT, torqueLimit)

    def SetHighestLimitTemperature(self, Id, highestLimitTemp):
        self._write_byte(Id, Registers.HIGHEST_LIMIT_TEMP, highestLimitTemp)

    def GetHighestLimitTemperature(self, Id):
        return self._read_byte(Id, Registers.HIGHEST_LIMIT_TEMP)

    #Returns the present position in bits. Depending on the model, it coulb be in [0,1023] or [0, 4095]
    def GetPresentPosition(self, Id): 
        return self._read_word(Id, Registers.PRESENT_POSITION)

    def GetRegistersValues(self, Id):

        print "Print registers of " + str(Id)
        print "Torque Limit:  " + str(self._read_word(Id, Registers.TORQUE_LIMIT))
        print "Moving speed:  " + str(self._read_word(Id, Registers.MOVING_SPEED))
        print "Torque enable:  " + str(self._read_byte(Id, Registers.TORQUE_ENABLE))
        print "Status return level:  " + str(self._read_byte(Id, Registers.STATUS_RETURN_LEVEL))
        print "CW angle Limit:  " + str(self._read_word(Id, Registers.CW_ANGLE_LIMIT))
        print "CCW angle Limit:  " + str(self._read_word(Id, Registers.CCW_ANGLE_LIMIT))
        print "Highest Limit Temp: " + str(self._read_byte(Id, Registers.HIGHEST_LIMIT_TEMP))
        print "   " 


