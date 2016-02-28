import serial, time

class Cmd():
    DRIVE_FORWARD_M1 = 0
    DRIVE_BACKWARDS_M1 = 1
    SET_MINIMUM_MAIN_VOLTAGE = 2
    SET_MAXIMUM_MAIN_VOLTAGE = 3
    DRIVE_FORWARD_M2 = 4
    DRIVE_BACKWARDS_M2 = 5
    DRIVE_M1 = 6
    DRIVE_M2 = 7
    DRIVE_FORWARD = 8
    DRIVE_BACKWARDS = 9
    TURN_RIGHT = 10
    TURN_LEFT = 11
    READ_Q_ENCODER_M1 = 16
    READ_Q_ENCODER_M2 = 17
    READ_M1_SPEED_TICKS_PER_SECOND = 18
    READ_M2_SPEED_TICKS_PER_SECOND = 19
    RESET_Q_ENCODERS_M1_AND_M2 = 20
    READ_MAIN_BATT_VOLTAGE = 24
    READ_M1_SPEED_125TH_SEC = 30
    READ_M2_SPEED_125TH_SEC = 31
    READ_MOTOR_CURRENTS = 49
    READ_TEMPERATURE = 82
	
#Initializes the serial port
def Open(comport, rate):
    global port
    port = serial.Serial(comport, rate, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout = 0.1);
    return
def Close():
    port.close()

#Drives forward motor M1. Values in [0,127]
def DriveForwardM1(address, value):
    commandBytes = bytearray([address, Cmd.DRIVE_FORWARD_M1, value, 0])
    commandBytes[3] = ((commandBytes[0] + commandBytes[1] + commandBytes[2]) & 0x7F)
    port.write(commandBytes)

#Drives backwards motor M1. Values in [0,127]
def DriveBackwardsM1(address, value):
    commandBytes = bytearray([address, Cmd.DRIVE_BACKWARDS_M1, value, 0])
    commandBytes[3] = ((commandBytes[0] + commandBytes[1] + commandBytes[2]) & 0x7F)
    port.write(commandBytes)

#Drives forward motor M2. Values in [0,127]
def DriveForwardM2(address, value):
    commandBytes = bytearray([address, Cmd.DRIVE_FORWARD_M2, value, 0])
    commandBytes[3] = ((commandBytes[0] + commandBytes[1] + commandBytes[2]) & 0x7F)
    port.write(commandBytes)

#Drives backwards motor M2. Values in [0,127]
def DriveBackwardsM2(address, value):
    commandBytes = bytearray([address, Cmd.DRIVE_BACKWARDS_M2, value, 0])
    commandBytes[3] = ((commandBytes[0] + commandBytes[1] + commandBytes[2]) & 0x7F)
    port.write(commandBytes)

#Reads the encoder counting, left or right depending on the value of "command", which should be Cmd.READ_Q_ENCODER_M1 or Cmd.READ_Q_ENCODER_M2
def __ReadQEncoder(address, command):
    commandBytes = bytearray([address, command])
    port.write(commandBytes)
    respStr = port.read(6)
    if len(respStr) != 6:
        print "Error while trying to read encoder: received packet must have 6 bytes :'("
        print "Roboclaw command used: " + str(command)            
        return 0
    respBytes = bytearray(respStr)
    result = (respBytes[0] << 24) + (respBytes[1] << 16) + (respBytes[2] << 8) + respBytes[3]
    if(respBytes[0] & 0x80):
        result -= 0xFFFFFFFF
    return result

#Reads the quadrature encoder of motor M1. Returns a signed integer
def ReadQEncoderM1(address):
    return __ReadQEncoder(address, Cmd.READ_Q_ENCODER_M1)

#Reads the quadrature encoder of motor M2. Returns a signed integer
def ReadQEncoderM2(address):
    return __ReadQEncoder(address, Cmd.READ_Q_ENCODER_M2)

#Reads encoder counter speed, left or right depending on "command", which should be Cmd.READ_M1_SPEED_TICKS_PER_SECOND or Cmd.READ_M2_SPEED_TICKS_PER_SECOND
#Returns ticks_per_second as a signed integer
def __ReadSpeed(address, command):
    commandBytes = bytearray([address, command])
    port.write(commandBytes)
    respStr = port.read(6)
    if len(respStr) != 6:
        print "Error while trying to read encoder speed: received packet must have 6 bytes :'("
        print "Roboclaw command used: " + str(command)            
        return 0
    respBytes = bytearray(respStr)
    result = (respBytes[0] << 24) + (respBytes[1] << 16) + (respBytes[2] << 8) + respBytes[3]
    if(respBytes[0] & 0x80):
        result -= 0xFFFFFFFF
    return result

#Read M1 encoder speed. Returns ticks_per_second
def ReadSpeedM1(address):
    return __ReadSpeed(address, Cmd.READ_M1_SPEED_TICKS_PER_SECOND)

#Read M2 encoder speed. Returns ticks_per_second
def ReadSpeedM2(address):
    return __ReadSpeed(address, Cmd.READ_M2_SPEED_TICKS_PER_SECOND)

#Resets Both encoders
def ResetQuadratureEncoders(address):
    commandBytes = bytearray([address, Cmd.RESET_Q_ENCODERS_M1_AND_M2, 0])
    commandBytes[2] = ((commandBytes[0] + commandBytes[1]) & 0x7F)
    port.write(commandBytes)

#Reads the main battery voltage. Returns voltage as a float value in [V]
def ReadMainBattVoltage(address):
    commandBytes = bytearray([address, Cmd.READ_MAIN_BATT_VOLTAGE])
    port.write(commandBytes)
    respStr = port.read(3)
    if len(respStr) != 3:
        print "Error while trying to read main battery level: received packet must have 3 bytes :'("
        print "Roboclaw command used: " + str(Cmd.READ_MAIN_BATT_VOLTAGE)            
        return 0
    respBytes = bytearray(respStr)
    result = (respBytes[0] << 8) + respBytes[1]
    result = float(result) * 0.1
    return result

#Reads the currents drawn by each motor. Returns an array of two floats: [Current_M1, Current_M2]
def ReadMotorCurrents(address):
    commandBytes = bytearray([address, Cmd.READ_MOTOR_CURRENTS])
    port.write(commandBytes)
    respStr = port.read(5)
    if len(respStr) != 5:
        print "Error while trying to read drawn current: received packet must have 5 bytes :'("
        print "Roboclaw command used: " + str(Cmd.READ_MOTOR_CURRENTS)            
        return [0, 0]
    respBytes = bytearray(respStr)
    current1 = float((respBytes[0] << 8) + respBytes[1]) * 0.01
    current2 = float((respBytes[2] << 8) + respBytes[3]) * 0.01
    return [current1, current2]
    
#Reads the current drawn by motor M1. Returns a float in [A]
def ReadMotorCurrentM1(address):
    return ReadMotorCurrents(address)[0]

#Reads the current drawn by motor M2. Returns a float in [A]
def ReadMotorCurrentM2(address):
    return ReadMotorCurrents(address)[1]

#Read the total current being drawn by both motors. Returns a float in [A]

def ReadMotorCurrentTotal(address):
    currents = ReadMotorCurrents(address)
    return currents[0] + currents[1]

#Read the current temperatur. Returns a float in [K]
def ReadTemperature(address):
    commandBytes = bytearray([address, Cmd.READ_TEMPERATURE])
    port.write(commandBytes)
    respStr = port.read(3)
    if len(respStr) != 3:
        print "Error while trying to read temperature: received packet must have 3 bytes :'("
        print "Roboclaw command used: " + str(Cmd.READ_TEMPERATURE)            
        return 0
    respBytes = bytearray(respStr)
    return float((respBytes[0] << 8) + respBytes[1]) * 0.1 + 273.15
