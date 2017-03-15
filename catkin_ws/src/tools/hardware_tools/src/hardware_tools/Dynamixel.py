import serial, time

class Registers():
    MODEL_NUMBER                = 0
    FIRMWARE_VERSION            = 2
    ID                          = 3
    BAUD_RATE                   = 4
    RETURN_DELAY_TIME           = 5
    CW_ANGLE_LIMIT              = 6
    CCW_ANGLE_LIMIT             = 8
    HIGHEST_LIMIT_TEMP          = 11
    LOWEST_LIMIT_VOLT           = 12
    HIGHEST_LIMIT_VOLT          = 13
    MAX_TORQUE                  = 14
    STATUS_RETURN_LEVEL         = 16
    ALARM_LED                   = 17
    ALARM_SHUTDOWN              = 18
    TORQUE_ENABLE               = 24
    LED                         = 25
    CW_COMPLIANCE_MARGIN        = 26
    CCW_COMPLIANCE_MARGIN       = 27
    CW_COMPLIANCE_SLOPE         = 28
    CCW_COMPLIANCE_SLOPE        = 29
    GOAL_POSITION               = 30
    MOVING_SPEED                = 32
    TORQUE_LIMIT                = 34
    PRESENT_POSITION            = 36
    PRESENT_SPEED               = 38
    PRESENT_LOAD                = 40
    PRESENT_VOLTAGE             = 42
    PRESENT_TEMPERATURE         = 43
    REGISTERED_INSTRUCTION      = 44
    MOVING                      = 46
    LOCK                        = 47
    PUNCH                       = 48
    CURRENT                     = 68


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
        #respStr = self.port.read(8)
        #respBytes = bytearray(respStr)
        #if respBytes[4] != 00000000: #If there is an error show this
        #    print "Error #: " + str(respBytes[4])

    def _write_word(self, Id, address, value): #Value should be a 16-bit data
        valueL = value & 0xFF
        valueH = (value >> 8) & 0xFF
        data = bytearray([255, 255, Id, 5, 3, address, valueL, valueH, 0])
        data[8] = ~((data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) & 0xFF) & 0xFF
        self.port.write(data)
        #respStr = self.port.read(8)
        #respBytes = bytearray(respStr)
        #if respBytes[4] != 00000000: #If there is an error show this
        #    print "Error #: " + str(respBytes[4])

    def _read_byte(self, Id, address): #reads the 8-bit data stored in address
        data = bytearray([255, 255, Id, 4, 2, address, 1, 0])
        data[7] = ~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF) & 0xFF
        self.port.write(data)

        respBytes = bytearray(self.port.read(3))
        if len(respBytes) < 3:
            print "Dynamixel: Error reading addr " + str(address) + ": No data available"
            return 0
        attempts = 4 #I have no f idea why this is the correct number of attempts
        while (respBytes[0] != 255 or respBytes[1] != 255 or respBytes[2] != Id) and attempts > 0:
            respBytes[0] = respBytes[1]
            respBytes[1] = respBytes[2]
            strTemp = self.port.read(1)
            if len(strTemp) != 1:
                strTemp = self.port.read(1)
                if len(strTemp) != 1:
                    continue
            respBytes[2] = ord(strTemp)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) +  " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0
        lenght = ord(self.port.read(1))

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0
        error = ord(self.port.read(1))

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) +  " id:" + str(Id) +": Max attempt exceeded for reading"
            return 0
        value = ord(self.port.read(1))

        self.port.read(self.port.inWaiting())

        #if error != 0:
        #    print "Error #: " + str(error) + "  ID: " + str(Id)

        return value

    def _read_word(self, Id, address): #reads the 16-bit data stored in address and address+1
        data = bytearray([255, 255, Id, 4, 2, address, 2, 0])
        data[7] = (~((data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF))& 0xFF
        self.port.write(data)

        respBytes = bytearray(self.port.read(3))
        if len(respBytes) == 0:
            respBytes = bytearray(self.port.read(3))
        if len(respBytes) < 3:
            print "Dynamixel: Error reading addr " + str(address) + " Id= " + str(Id) + ": No data available"
            return 0
        attempts = 4 #I have no f idea why this is the correct number of attempts
        while (respBytes[0] != 255 or respBytes[1] != 255 or respBytes[2] != Id) and attempts > 0:
            respBytes[0] = respBytes[1]
            respBytes[1] = respBytes[2]
            strTemp = self.port.read(1)
            if len(strTemp) != 1:
                strTemp = self.port.read(1)
                if len(strTemp) != 1:
                    continue
            respBytes[2] = ord(strTemp)
            attempts -= 1

        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0
        lenght = ord(self.port.read(1))

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0
        error = ord(self.port.read(1))

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0
        lValue = ord(self.port.read(1))

        attempts = 4
        while self.port.inWaiting() < 1 and attempts >0:
            time.sleep(0.001)
            attempts -= 1
        if attempts <= 0:
            print "Dynamixel: Error reading addr " + str(address) + " id:" + str(Id) + ": Max attempt exceeded for reading"
            return 0
        hValue = ord(self.port.read(1))

        self.port.read(self.port.inWaiting())

        #if error != 0:
        #    print "Error #: " + str(error) + "  ID: " + str(Id)

        return ((hValue << 8) + lValue)

    #Each servo has a status return level, nevertheless, here it's assumed that all servos wired to the same bus will have the same status-return-level
    #This function, with no arguments, returns the StatusReturnLevel that is suposed to be set in all servos wired to the same bus
    #A similar function, but with an ID as an argument, returns the StatusReturnLevel of the servo with such ID
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

    def SetTorqueDisable(self, Id):
        self._write_byte(Id, Registers.TORQUE_ENABLE, False)

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

    def SetMaxTorque(self, Id, maxTorque):
        self._write_word(Id, Registers.MAX_TORQUE, maxTorque)

    def GetMaxTorque(self, Id, maxTorque):
        return self._read_word(Id, Registers.MAX_TORQUE)

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

    def GetPresentVoltage(self, Id):
        return self._read_byte(Id, Registers.PRESENT_VOLTAGE)

    def GetPresentLoad(self, Id):
        return self._read_word(Id, Registers.PRESENT_LOAD)

    def SetDGain(self, Id, DGain):
        self._write_byte(Id, Registers.CW_COMPLIANCE_MARGIN, DGain)

    def SetIGain(self, Id, IGain):
        self._write_byte(Id, Registers.CCW_COMPLIANCE_MARGIN, IGain)

    def SetPGain(self, Id, PGain):
        self._write_byte(Id, Registers.CW_COMPLIANCE_SLOPE, PGain)

    def SetCWComplianceMargin(self, Id, ComMargCW):
        self._write_byte(Id, Registers.CW_COMPLIANCE_MARGIN, ComMargCW)

    def SetCCWComplianceMargin(self, Id, ComMargCCW):
        self._write_byte(Id, Registers.CCW_COMPLIANCE_MARGIN, ComMargCCW)

    def SetCWComplianceSlope(self, Id, ComSlopeCW):
        self._write_byte(Id, Registers.CW_COMPLIANCE_SLOPE, ComSlopeCW)

    def SetCCWComplianceSlope(self, Id, ComSlopeCCW):
        self._write_byte(Id, Registers.CCW_COMPLIANCE_SLOPE, ComSlopeCCW)

    def SetAlarmShutdown(self, Id, alarmShutdown):
        self._write_word(Id, Registers.ALARM_SHUTDOWN, alarmShutdown)


    def GetRegistersValues(self, Id):

        print "Print registers of " + str(Id)
        print "Torque Limit:  " + str(self._read_word(Id, Registers.TORQUE_LIMIT))
        print "Moving speed:  " + str(self._read_word(Id, Registers.MOVING_SPEED))
        print "Torque enable:  " + str(self._read_byte(Id, Registers.TORQUE_ENABLE))
        print "Status return level:  " + str(self._read_byte(Id, Registers.STATUS_RETURN_LEVEL))
        print "CW angle Limit:  " + str(self._read_word(Id, Registers.CW_ANGLE_LIMIT))
        print "CCW angle Limit:  " + str(self._read_word(Id, Registers.CCW_ANGLE_LIMIT))
        print "Highest Limit Temp: " + str(self._read_byte(Id, Registers.HIGHEST_LIMIT_TEMP))
        print "Batery: " + str(float(self._read_byte(Id, Registers.PRESENT_VOLTAGE)/10)) + " [V]"
        print "Present temperature: " + str(self._read_byte(Id, Registers.PRESENT_TEMPERATURE)) + " [C]"
        print "Max Torque: " + str(self._read_word(Id, Registers.MAX_TORQUE)) + " " + str(int((self._read_word(Id, Registers.MAX_TORQUE))/1023*100)) + "%"
        print "Alarm led: " + str(self._read_byte(Id, Registers.ALARM_LED))
        print "Alarm Shutdown: " + str(self._read_byte(Id, Registers.ALARM_SHUTDOWN))
        print "   "



