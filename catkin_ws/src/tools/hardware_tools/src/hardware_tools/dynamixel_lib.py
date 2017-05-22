#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----  Edited by Edgar Vazquez --------
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

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

    # Protocol version
    PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

    # Default setting
    DXL_ID                      = 1                             # Dynamixel ID: 1
    BAUDRATE                    = 1000000

    # Protocol version
    TORQUE_ENABLE               = 1                             # Value for enabling the Torque
    TORQUE_DISABLE              = 0                             # Value for disabling the torque
    DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MOVING_STATUS_THRESHOLD = 10                            # Dynamixel moving status threshold

    ESC_ASCII_VALUE             = 0x1b

    COMM_SUCCESS                = 0                             # Communication Success result value
    COMM_TX_FAIL                = -1001                         # Communication Tx Failed


class DynamixelMan:
    'Class for communicating with a set of dynamixel servomotors connected to the same bus'
    def __init__(self, portName, baudrate):
        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(portName)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        #Open port
        if dynamixel.openPort(self.port_num):
            print("HardwareTools-Dynamixel.->Succeeded to open the port!")
        else:
            print("HardwareTools-Dynamixel.->Failed to open the port!")
            print("HardwareTools-Dynamixel.->Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, baudrate):
            print("HardwareTools-Dynamixel.->Succeeded to change the baudrate!")
        else:
            print("HardwareTools-Dynamixel.->Failed to change the baudrate!")
            print("HardwareTools-Dynamixel.->Press any key to terminate...")
            getch()
            quit()

    def Close(self):
        dynamixel.closePort(self.port_num)

    ###
    ###Methods for writing and reading bytes
    ###
    def GetTorqueEnable(self, Id):
        try:
            torque_enable = dynamixel.read1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.TORQUE_ENABLE)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return torque_enable
        except :
            print "Oops!  Problem reading Torque..."
            return -3141592

    def GetCWAngleLimit(self, Id):
        try:
            cw_angle_limit = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CW_ANGLE_LIMIT)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return cw_angle_limit
        except :
            print "Oops!  Problem reading..."

    def SetCWAngleLimit(self, Id, angleLimit):
        try:
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CW_ANGLE_LIMIT, angleLimit)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing ..."

    def GetCCWAngleLimit(self, Id):
        try:
            ccw_angle_limit = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CCW_ANGLE_LIMIT)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return ccw_angle_limit
        except :
            print "Oops!  Problem reading..."

    def SetCCWAngleLimit(self, Id, angleLimit):
        try:
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CCW_ANGLE_LIMIT, angleLimit)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing ..."

    def SetHighestLimitTemperature(self, Id, highestLimitTemp):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.HIGHEST_LIMIT_TEMP, highestLimitTemp)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing..."


    def SetTorqueEnable(self, Id, enable):
        # Enable Dynamixel Torque
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.TORQUE_ENABLE, enable)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing..."

    def SetTorqueDisable(self, Id):
        # Enable Dynamixel Torque
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.TORQUE_ENABLE, 0)
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.TORQUE_ENABLE, 0)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing..."


    #Goal position could be in [0,1023] or [0,4095] depending on the servo model
    def GetGoalPosition(self, Id):
        try:
            goal_position = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.GOAL_POSITION)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return goal_position
        except :
            print "Oops!  Problem reading..."

    def SetGoalPosition(self, Id, goalPose):
        try:
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.GOAL_POSITION, goalPose)
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.GOAL_POSITION, goalPose)
            if dynamixel.getLastTxRxResult(self.port_num, Registers.PROTOCOL_VERSION) != Registers.COMM_SUCCESS:
                #dynamixel.printTxRxResult(Registers.PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, Registers.PROTOCOL_VERSION))
                return False
            elif dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            else:
                return True
        except :
            print "Oops!  Problem writing..."
            return False


    def GetMovingSpeed(self, Id):
        try:
            moving_speed = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.MOVING_SPEED)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return moving_speed
        except :
            print "Oops!  Problem reading moving speed..."

    def SetMovingSpeed(self, Id, movingSpeed):
        try:
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.MOVING_SPEED, movingSpeed)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
                return False
            else:
                return True
        except :
            print "Oops!  Problem writing moving speed..."

    def SetTorqueVale(self, Id, torqueValue, directionTurn):
        try:
            if directionTurn == True:
                torqueValue = torqueValue + 1024
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.MOVING_SPEED, torqueValue)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
                return False
            else:
                return True
        except :
            print "Oops!  Problem writing torque value..."

    def SetMaxTorque(self, Id, maxTorque):
        try:
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.MAX_TORQUE, maxTorque)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing max torque..."

    def GetMaxTorque(self, Id, maxTorque):
        try:
            max_torque = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.MAX_TORQUE)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return max_torque
        except :
            print "Oops!  Problem reading max torque..."

    def GetTorqueLimit(self, Id):
        try:
            torque_limit = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.TORQUE_LIMIT)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return torque_limit
        except :
            print "Oops!  Problem reading Torque limit..."

    def SetTorqueLimit(self, Id, torqueLimit):
        try:
            dynamixel.write2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.TORQUE_LIMIT, torqueLimit)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing torque limit..."

    #Returns the present position in bits. Depending on the model, it coulb be in [0,1023] or [0, 4095]
    def GetPresentPosition(self, Id):
        try:
            present_position = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.PRESENT_POSITION)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            else:
                return present_position
        except :
            print "Oops!  Problem writing goal position..."
            return -3141592
        # Read present position


    def GetPresentVoltage(self, Id):
        try:
            present_voltaje = dynamixel.read1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.PRESENT_VOLTAGE)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return present_voltaje
        except :
            print "Oops!  Problem writing goal position..."

    def GetPresentLoad(self, Id):
        try:
            present_load = dynamixel.read2ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.PRESENT_LOAD)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
            return present_load
        except :
            print "Oops!  Problem writing goal position..."

    def SetDGain(self, Id, DGain):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CW_COMPLIANCE_MARGIN, DGain)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetIGain(self, Id, IGain):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CCW_COMPLIANCE_MARGIN, IGain)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetPGain(self, Id, PGain):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CW_COMPLIANCE_SLOPE, PGain)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetCWComplianceMargin(self, Id, ComMargCW):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CW_COMPLIANCE_MARGIN, ComMargCW)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetCCWComplianceMargin(self, Id, ComMargCCW):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CCW_COMPLIANCE_MARGIN, ComMargCCW)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetCWComplianceSlope(self, Id, ComSlopeCW):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CW_COMPLIANCE_SLOPE, ComSlopeCW)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetCCWComplianceSlope(self, Id, ComSlopeCCW):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.CCW_COMPLIANCE_SLOPE, ComSlopeCCW)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

    def SetAlarmShutdown(self, Id, alarmShutdown):
        try:
            dynamixel.write1ByteTxRx(self.port_num, Registers.PROTOCOL_VERSION, Id, Registers.ALARM_SHUTDOWN, alarmShutdown)
            if dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(Registers.PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, Registers.PROTOCOL_VERSION))
        except :
            print "Oops!  Problem writing goal position..."

