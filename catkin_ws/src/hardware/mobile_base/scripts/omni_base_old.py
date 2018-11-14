#!/usr/bin/env python
###THIS NODE HAS SIMILAR FUNCTIONALITY TO OMNI_BASE_NODE, BUT IT USES THE SPEED CONTROL
###ALREADY IMPLEMENTED IN THE ROBOCLAW BOARD. THE SIMULATION OPTION HAS BEEN SUPRESSED
###FOR SIMULATION YOU SHOULD USE THE OMNI_BASE_SIMUL NODE. 
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import roboclaw
import tf

base_diameter = 0.52;
rc_address_frontal = 0x80;
rc_address_lateral  = 0x80; 
rc_frontal = roboclaw.Roboclaw("/dev/ttyACM0", 38400); #Roboclaw controling motors for frontal movement (left and right)
rc_lateral = roboclaw.Roboclaw("/dev/ttyACM1", 38400); #Roboclaw controling motors for lateral movement (front and rear)
rc_acceleration = 1000000;
global simul;
simul = False;

def print_help():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def check_speed_ranges(s_left, s_right, s_front, s_rear): #speeds: left, right, front and rear
    max_value_frontal = max(abs(s_left), abs(s_right));
    max_value_lateral = max(abs(s_front), abs(s_rear));

    if max_value_lateral > 1.0:
        print "MobileBase.->Warning! front and rear speeds should not be greater than 1.0. Normalized speeds used instead"
        s_left  /= max_value_lateral;
        s_right /= max_value_lateral;
        s_front /= max_value_lateral;
        s_rear  /= max_value_lateral;
        max_value_frontal/= max_value_lateral;
    if max_value_frontal > 2.0:
        s_left  = s_left  / max_value_frontal * 2.0;
        s_right = s_right / max_value_frontal * 2.0;
        s_front = s_front / max_value_frontal * 2.0;
        s_rear  = s_rear  / max_value_frontal * 2.0;
    #print "Corrected speeds: " + str(s_left) + "\t" + str(s_right) + "\t" + str(s_front) + "\t" + str(s_rear)
    return (s_left, s_right, s_front, s_rear);

def callback_stop(msg):
    global speed_left, speed_right, speed_front, speed_rear, new_data;
    speed_left  = 0;
    speed_right = 0;
    speed_front = 0;
    speed_rear  = 0;
    new_data = True;

def callback_speeds(msg):
    global speed_left, speed_right, speed_front, speed_rear, new_data;
    speed_left  = msg.data[0];
    speed_right = msg.data[1];
    speed_front = (speed_right - speed_left)/2.0;
    speed_rear  = (speed_left - speed_right)/2.0;
    new_data = True;

def callback_cmd_vel(msg):
    global speed_left, speed_right, speed_front, speed_rear, new_data;
    speed_left  = msg.linear.x - msg.angular.z * base_diameter/2.0
    speed_right = msg.linear.x + msg.angular.z * base_diameter/2.0
    speed_front = msg.linear.y + msg.angular.z * base_diameter/2.0
    speed_rear  = msg.linear.y - msg.angular.z * base_diameter/2.0
    new_data = True;

def callback_simulated(msg):
    global simul;
    simul = msg.data;
        

def calculate_odometry(pos_x, pos_y, pos_theta, enc_left, enc_right, enc_front, enc_rear):
    #TICKS_PER_METER_LATERAL = 336857.5; #Ticks per meter for the slow motors (front and rear)
    #TICKS_PER_METER_FRONTAL = 158891.2; #Ticks per meter for the fast motors (left and right)
    TICKS_PER_METER_LATERAL = 158891.2; #Ticks per meter for the slow motors (front and rear)
    TICKS_PER_METER_FRONTAL = 164352.1; #Ticks per meter for the fast motors (left and right)
    enc_left  /= TICKS_PER_METER_FRONTAL;
    enc_right /= TICKS_PER_METER_FRONTAL;
    enc_front /= TICKS_PER_METER_LATERAL;
    enc_rear  /= TICKS_PER_METER_LATERAL;

    delta_theta = (enc_right - enc_left + enc_front - enc_rear)/base_diameter/2.0
    if math.fabs(delta_theta) >= 0.00001:
        rg_x = (enc_left + enc_right)/(2*delta_theta)
        rg_y = (enc_rear + enc_front)/(2*delta_theta)
        delta_x = rg_x*math.sin(delta_theta)     + rg_y*(1-math.cos(delta_theta))
        delta_y = rg_x*(1-math.cos(delta_theta)) + rg_y*math.sin(delta_theta)
    else:
        delta_x = (enc_left + enc_right)/2
        delta_y = (enc_rear + enc_front)/2
    pos_x += delta_x * math.cos(pos_theta) - delta_y * math.sin(pos_theta)
    pos_y += delta_x * math.sin(pos_theta) + delta_y * math.cos(pos_theta)
    pos_theta += delta_theta
    return (pos_x, pos_y, pos_theta);



def main():
    print "MobileBase.-> INITIALIZING THE AMAZING MOBILE BASE NODE BY MARCOSOFT..."

    rospy.init_node("mobile_base");
    
    port_name_frontal = "/dev/ttyACM0";
    port_name_lateral = "/dev/ttyACM1";
    global simul
    simul = False
    
    if rospy.has_param('~simul'):
        simul = rospy.get_param('~simul')
    
    if rospy.has_param('~port1'):
        port_name_frontal = rospy.get_param('~port1')
    elif not simul:
        print_help();
        sys.exit();
    if rospy.has_param('~port2'):
        port_name_lateral = rospy.get_param('~port2')
    elif not simul:
        print_help();
        sys.exit();

    #ROS CONNECTION
    pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
    subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop, queue_size=1);
    subSpeeds  = rospy.Subscriber("/hardware/mobile_base/speeds",  Float32MultiArray, callback_speeds, queue_size=1);
    subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callback_cmd_vel, queue_size=1);
    subSimul   = rospy.Subscriber("/simulated", Bool, callback_simulated, queue_size = 1);
    br   = tf.TransformBroadcaster()
    rate = rospy.Rate(30);

    #ROBOCLAW CONNECTION
    rc_frontal.comport = port_name_frontal;
    rc_lateral.comport = port_name_lateral;
    if not simul and rc_frontal.Open() != 1:
        print "MobileBase.-> Cannot open Roboclaw for left and right motors on " + rc_frontal.comport;
        return;
    if not simul and rc_lateral.Open() != 1:
        print "MobileBase.-> Cannot open Roboclaw for front and rear motors on " + rc_lateral.comport;
        return;
    print "MobileBase.-> Roboclaw frontal open on port " + rc_frontal.comport;
    print "MobileBase.-> Roboclaw lateral open on port " + rc_lateral.comport;
    global QPPS_LEFT 
    global QPPS_RIGHT
    global QPPS_FRONT
    global QPPS_REAR 
    global speed_left, speed_right, speed_front, speed_rear, new_data
    if not simul:
        rc_frontal.ResetEncoders(rc_address_frontal);
        rc_lateral.ResetEncoders(rc_address_lateral);
        #ROBOCLAW CONFIGURATION CONSTANTS
        pos_PID_left  = rc_frontal.ReadM1PositionPID(rc_address_frontal);
        pos_PID_right = rc_frontal.ReadM2PositionPID(rc_address_frontal);
        pos_PID_front = rc_lateral.ReadM1PositionPID(rc_address_lateral);
        pos_PID_rear  = rc_lateral.ReadM2PositionPID(rc_address_lateral);
        vel_PID_left  = rc_frontal.ReadM1VelocityPID(rc_address_frontal);
        vel_PID_right = rc_frontal.ReadM2VelocityPID(rc_address_frontal);
        vel_PID_front = rc_lateral.ReadM1VelocityPID(rc_address_lateral);
        vel_PID_rear  = rc_lateral.ReadM2VelocityPID(rc_address_lateral);
        print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos";
        print "MobileBase.->Left Motor:  " + str(pos_PID_left );
        print "MobileBase.->Right Motor: " + str(pos_PID_right);
        print "MobileBase.->Front Motor: " + str(pos_PID_front);
        print "MobileBase.->Rear Motor:  " + str(pos_PID_rear );
        print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS"; #QPPS = speed in ticks/s when motor is at full speed
        print "MobileBase.->Left Motor:  " + str(vel_PID_left );
        print "MobileBase.->Right Motor: " + str(vel_PID_right);
        print "MobileBase.->Front Motor: " + str(vel_PID_front);
        print "MobileBase.->Left Motor:  " + str(vel_PID_rear );
        QPPS_LEFT  = vel_PID_left[4];
        QPPS_RIGHT = vel_PID_right[4];
        QPPS_FRONT = vel_PID_front[4];
        QPPS_REAR  = vel_PID_rear[4];
        if QPPS_LEFT != QPPS_RIGHT or QPPS_REAR != QPPS_FRONT:
            print "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur."
        if rc_frontal.ReadPWMMode(rc_address_frontal)[1] == 1:
            print "MobileBase.->PWM Mode for left and right motors: Sign magnitude"
        else:
            print "MobileBase.->PWM Mode for left and right motors: Locked antiphase"
        if rc_lateral.ReadPWMMode(rc_address_lateral)[1] == 1:
            print "MobileBase.->PWM Mode for front and rear motors: Sign magnitude"
        else:
            print "MobileBase.->PWM Mode for front and rear motors: Locked antiphase"
    else:
        QPPS_LEFT  = 158891.2
        QPPS_RIGHT = 158891.2
        QPPS_FRONT = 158891.2
        QPPS_REAR  = 158891.2

    new_data = False;
    no_new_data_counter = 5;  #If no new speed data arrive after 5 cycles, motors are stopped.
    robot_x = 0;
    robot_y = 0;
    robot_t = 0;
    encoder_left  = 0;
    encoder_right = 0;
    encoder_front = 0;
    encoder_rear  = 0;
    encoder_last_left  = 0;
    encoder_last_right = 0;
    encoder_last_front = 0;
    encoder_last_rear  = 0;
    speed_left = 0
    speed_right = 0
    speed_front = 0
    speed_rear = 0

    while not rospy.is_shutdown():
        if not new_data:
            no_new_data_counter -= 1;
            if no_new_data_counter == 0:
                if not simul:
                    rc_frontal.ForwardM1(rc_address_frontal, 0);
                    rc_frontal.ForwardM2(rc_address_frontal, 0);
                    rc_lateral.ForwardM1(rc_address_lateral, 0);
                    rc_lateral.ForwardM2(rc_address_lateral, 0);
                else:
                    speed_left = 0
                    speed_right = 0
                    speed_front = 0
                    speed_rear = 0
            if no_new_data_counter < -1:
                no_new_data_counter = -1;
        else:
            new_data = False;
            no_new_data_counter = 5;
            (speed_left,speed_right,speed_front,speed_rear) = check_speed_ranges(speed_left,speed_right,speed_front,speed_rear);
            if not simul:
                #speed_left  =  int(speed_left  * 32767 * 16.0/35.0);                                              
                #speed_right =  int(speed_right * 32767 * 16.0/35.0);                                              
                #speed_front = -int(speed_front * 32767);                                                          
                #speed_rear  = -int(speed_rear  * 32767);                                                          
                #rc_frontal.DutyM1M2(rc_address_frontal, speed_left, speed_right);                                 
                #rc_lateral.DutyM1M2(rc_address_lateral, speed_front, speed_rear);
                speed_left  =  int(speed_left  * QPPS_LEFT  * 16.0/35.0);                               
                speed_right =  int(speed_right * QPPS_RIGHT * 16.0/35.0);                               
                speed_front = -int(speed_front * QPPS_FRONT * 16.0/35.0);                                           
                speed_rear  = -int(speed_rear  * QPPS_REAR * 16.0/35.0);                                            
                #rc_frontal.SpeedAccelM1M2(rc_address_frontal, rc_acceleration, speed_left, speed_right);
                #rc_lateral.SpeedAccelM1M2(rc_address_lateral, rc_acceleration, speed_front, speed_rear);
                try:
                    rc_frontal.SpeedM1(rc_address_frontal, speed_left);
                    rc_frontal.SpeedM2(rc_address_frontal, speed_right);
                except:
                    print "Mobile base.-> Error while writing speeds to roboclaw frontal"
                try:
                    rc_lateral.SpeedM1(rc_address_lateral, speed_front);
                    rc_lateral.SpeedM2(rc_address_lateral, speed_rear);
                except:
                    print "Mobile base.-> Error while writing speeds to roboclaw lateral"
        #Getting encoders for odometry calculation
        if not simul:
            encoder_left  =  rc_frontal.ReadEncM1(rc_address_frontal)[1];
            encoder_right =  rc_frontal.ReadEncM2(rc_address_frontal)[1];
            encoder_front = -rc_lateral.ReadEncM1(rc_address_lateral)[1];
            encoder_rear  = -rc_lateral.ReadEncM2(rc_address_lateral)[1];
            delta_left  = encoder_left  - encoder_last_left;
            delta_right = encoder_right - encoder_last_right;
            delta_front = encoder_front - encoder_last_front;
            delta_rear  = encoder_rear  - encoder_last_rear;
            encoder_last_left  = encoder_left 
            encoder_last_right = encoder_right
            encoder_last_front = encoder_front
            encoder_last_rear  = encoder_rear
            # print "Encoders delta: " + str(encoder_left) + "\t" + str(encoder_right);
        else:
            encoder_left = speed_left * 0.05 * QPPS_LEFT
            encoder_right = speed_right * 0.05 * QPPS_RIGHT
            encoder_front = speed_front * 0.05 * QPPS_FRONT
            encoder_rear = speed_rear * 0.05 * QPPS_REAR
            delta_left  = encoder_left;
            delta_right = encoder_right;
            delta_front = encoder_front;
            delta_rear  = encoder_rear;
        if abs(delta_left)<24000 and abs(delta_right)<24000 and abs(delta_front)<48000 and abs(delta_rear)<48000:
            (robot_x,robot_y,robot_t)=calculate_odometry(robot_x,robot_y,robot_t,delta_left,delta_right, delta_front, delta_rear);
        else:
            print "MobileBase.->Invalid encoder readings. OMFG!!!!!!!"
            print "Encoders delta: " + str(delta_left) + "\t" + str(delta_right) + "\t" + str(delta_front) + "\t" + str(delta_rear);

        quaternion = tf.transformations.quaternion_from_euler(0, 0, robot_t);
        br.sendTransform((robot_x, robot_y, 0), quaternion, rospy.Time.now(), "base_link", "odom");
        
        if not simul:
            pubBattery.publish(Float32(rc_frontal.ReadMainBatteryVoltage(rc_address_frontal)[1]));
        else:
            pubBattery.publish(Float32(12.0));
        rate.sleep();

    print "MobileBase.->Stopping motors..."
    if not simul:
        rc_frontal.ForwardM1(rc_address_frontal, 0);
        rc_frontal.ForwardM2(rc_address_frontal, 0);
        rc_lateral.ForwardM1(rc_address_lateral, 0);
        rc_lateral.ForwardM2(rc_address_lateral, 0);

if __name__ == '__main__':
    main();
