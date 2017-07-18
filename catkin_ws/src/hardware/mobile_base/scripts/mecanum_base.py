#!/usr/bin/env python
###THIS NODE HAS SIMILAR FUNCTIONALITY TO OMNI_BASE_NODE, BUT IT USES THE SPEED CONTROL
###ALREADY IMPLEMENTED IN THE ROBOCLAW BOARD. THE SIMULATION OPTION HAS BEEN SUPRESSED
###FOR SIMULATION YOU SHOULD USE THE OMNI_BASE_SIMUL NODE. 
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import roboclaw
import tf

base_diameter = 0.86;
rc_address_left  = 0x80;
rc_address_right = 0x80; 
rc_left  = roboclaw.Roboclaw("/dev/ttyACM0", 38400); #Roboclaw controling motors for frontal movement (left and right)
rc_right = roboclaw.Roboclaw("/dev/ttyACM1", 38400); #Roboclaw controling motors for lateral movement (front and rear)
rc_acceleration = 1000000;

def print_help():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def check_speed_ranges(s_left_f, s_left_r, s_right_f, s_right_r): #speeds: left-frontal, left-rear, right-frontal, right-rear
    max_value = max(abs(s_left_f), abs(s_left_r), abs(s_right_f), abs(s_right_r));

    if max_value > 2.0:  #This is the max value used in the omni_base for the fast_motors, which have the same reduction than these motors
        print "MobileBase.->Warning! speeds should not be greater than 1.0. Normalized speeds used instead"
        s_left_f  = s_left_f  / max_value * 2.0;
        s_left_r  = s_left_r  / max_value * 2.0;
        s_right_f = s_right_f / max_value * 2.0;
        s_right_r = s_right_r / max_value * 2.0;
    return (s_left_f, s_left_r, s_right_f, s_right_r);

def callback_stop(msg):
    global speed_left_f, speed_left_r, speed_right_f, speed_right_r, new_data;
    speed_left_f  = 0;
    speed_left_r  = 0;
    speed_right_f = 0;
    speed_right_r = 0;
    new_data = True;

def callback_speeds(msg):
    global speed_left_f, speed_left_r, speed_right_f, speed_right_r, new_data;
    speed_left_f  = msg.data[0];
    speed_left_r  = msg.data[0];
    speed_right_f = msg.data[1];
    speed_right_r = msg.data[1]
    new_data = True;
        
def callback_cmd_vel(msg):
    global speed_left_f, speed_left_r, speed_right_f, speed_right_r, new_data;
    speed_left_f  = msg.linear.x - msg.angular.z*base_diameter/2.0 - msg.linear.y/2.0/0.707;
    speed_left_r  = msg.linear.x - msg.angular.z*base_diameter/2.0 + msg.linear.y/2.0/0.707;
    speed_right_f = msg.linear.x + msg.angular.z*base_diameter/2.0 + msg.linear.y/2.0/0.707;
    speed_right_r = msg.linear.x + msg.angular.z*base_diameter/2.0 - msg.linear.y/2.0/0.707;
    new_data = True;
 
def calculate_odometry(pos_x, pos_y, pos_theta, enc_left_f, enc_left_r, enc_right_f, enc_right_r):
    TICKS_PER_METER = 123517.665; #Ticks per meter for the fast motors with the mecanum wheels
    enc_left_f  /= TICKS_PER_METER;
    enc_left_r  /= TICKS_PER_METER;
    enc_right_f /= TICKS_PER_METER;
    enc_right_r /= TICKS_PER_METER;

    delta_theta = (enc_right_f - enc_left_f + enc_right_r - enc_left_r) / base_diameter / 2.0;
    if math.fabs(delta_theta) >= 0.00001:
        rg_x = (enc_left_f + enc_right_f + enc_left_r + enc_right_r)/(4.0 * delta_theta);
        rg_y = (enc_right_f + enc_left_r - enc_right_r - enc_left_f)/(4.0 * delta_theta)*0.707;
        delta_x = rg_x * math.sin(delta_theta)      + rg_y * (1-math.cos(delta_theta));
        delta_y = rg_x * (1-math.cos(delta_theta))  + rg_y * math.sin(delta_theta);
    else:
        delta_x = (enc_left_f + enc_right_f + enc_left_r + enc_right_r)/4.0;
        delta_y = (enc_right_f + enc_left_r - enc_right_r - enc_left_f)/4.0*0.707;
    
    
    pos_x += delta_x * math.cos(pos_theta) - delta_y * math.sin(pos_theta)
    pos_y += delta_x * math.sin(pos_theta) + delta_y * math.cos(pos_theta)
    pos_theta += delta_theta
    return (pos_x, pos_y, pos_theta);


def main(port_name_left, port_name_right):
    print "MobileBase.-> INITIALIZING THE AMAZING MOBILE BASE NODE BY MARCOSOFT..."

    #ROS CONNECTION
    rospy.init_node("mobile_base");
    pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
    subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop, queue_size=1);
    subSpeeds  = rospy.Subscriber("/hardware/mobile_base/speeds",  Float32MultiArray, callback_speeds, queue_size=1);
    subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callback_cmd_vel, queue_size=1);
    br   = tf.TransformBroadcaster()
    rate = rospy.Rate(30);

    #ROBOCLAW CONNECTION
    rc_left.comport  = port_name_left;
    rc_right.comport = port_name_right;
    if rc_left.Open() != 1:
        print "MobileBase.-> Cannot open Roboclaw for left_front and left_rear motors on " + rc_left.comport;
        return;
    if rc_right.Open() != 1:
        print "MobileBase.-> Cannot open Roboclaw for right_front and right_rear motors on " + rc_right.comport;
        return;
    print "MobileBase.-> Roboclaw left open on port " + rc_left.comport;
    print "MobileBase.-> Roboclaw right open on port " + rc_right.comport;
    rc_left.ResetEncoders(rc_address_left);
    rc_right.ResetEncoders(rc_address_right);

    #ROBOCLAW CONFIGURATION CONSTANTS
    pos_PID_left_f  = rc_left.ReadM2PositionPID(rc_address_left);
    pos_PID_left_r  = rc_left.ReadM1PositionPID(rc_address_left);
    pos_PID_right_f = rc_right.ReadM2PositionPID(rc_address_right);
    pos_PID_right_r = rc_right.ReadM1PositionPID(rc_address_right);
    vel_PID_left_f  = rc_left.ReadM2VelocityPID(rc_address_left);
    vel_PID_left_r  = rc_left.ReadM1VelocityPID(rc_address_left);
    vel_PID_right_f = rc_right.ReadM2VelocityPID(rc_address_right);
    vel_PID_right_r = rc_right.ReadM1VelocityPID(rc_address_right);
    print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos";
    print "MobileBase.->left_f  Motor:  " + str(pos_PID_left_f );
    print "MobileBase.->left_r  Motor:  " + str(pos_PID_left_r );
    print "MobileBase.->right_f Motor:  " + str(pos_PID_right_f);
    print "MobileBase.->right_r Motor:  " + str(pos_PID_right_r);
    print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS"; #QPPS = speed in ticks/s when motor is at full speed
    print "MobileBase.->left_f  Motor:  " + str(vel_PID_left_f );
    print "MobileBase.->left_r  Motor:  " + str(vel_PID_left_r );
    print "MobileBase.->right_f Motor:  " + str(vel_PID_right_f);
    print "MobileBase.->right_r Motor:  " + str(vel_PID_right_r);
    QPPS_LEFT_F  = vel_PID_left_f[4];
    QPPS_LEFT_R  = vel_PID_left_r[4];
    QPPS_RIGHT_F = vel_PID_right_f[4];
    QPPS_RIGHT_R = vel_PID_right_r[4];
    if QPPS_LEFT_F != QPPS_LEFT_R or QPPS_RIGHT_F != QPPS_RIGHT_R:
        print "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur."
    if rc_left.ReadPWMMode(rc_address_left)[1] == 1:
        print "MobileBase.->PWM Mode for left-front and left-rear motors: Sign magnitude"
    else:
        print "MobileBase.->PWM Mode for left-front and left-rear motors: Locked antiphase"
    if rc_right.ReadPWMMode(rc_address_right)[1] == 1:
        print "MobileBase.->PWM Mode for right-front and right-rear motors: Sign magnitude"
    else:
        print "MobileBase.->PWM Mode for right-front and right-rear motors: Locked antiphase"
    
    global speed_left_f, speed_left_r, speed_right_f, speed_right_r, new_data;
    new_data = False;
    no_new_data_counter = 5;  #If no new speed data arrive after 5 cycles, motors are stopped.
    robot_x = 0;
    robot_y = 0;
    robot_t = 0;
    encoder_left_f  = 0;
    encoder_left_r  = 0;
    encoder_right_f = 0;
    encoder_right_r = 0;
    encoder_last_left_f  = 0;
    encoder_last_left_r  = 0;
    encoder_last_right_f = 0;
    encoder_last_right_r = 0;
        
    while not rospy.is_shutdown():
        if not new_data:
            no_new_data_counter -= 1;
            if no_new_data_counter == 0:
                rc_left.ForwardM1(rc_address_left, 0);
                rc_left.ForwardM2(rc_address_left, 0);
                rc_right.ForwardM1(rc_address_right, 0);
                rc_right.ForwardM2(rc_address_right, 0);
            if no_new_data_counter < -1:
                no_new_data_counter = -1;
        else:
            new_data = False;
            no_new_data_counter = 5;
            (speed_left_f,speed_left_r,speed_right_f,speed_right_r) = check_speed_ranges(speed_left_f,speed_left_r,speed_right_f,speed_right_r);
            speed_left_f  =  int(speed_left_f  * QPPS_LEFT_F  * 16.0/35.0);  #This factor is left for compatibility with the omni_base.py 
            speed_left_r  =  int(speed_left_r  * QPPS_LEFT_R  * 16.0/35.0);
            speed_right_f = -int(speed_right_f * QPPS_RIGHT_F * 16.0/35.0);                                           
            speed_right_r = -int(speed_right_r * QPPS_RIGHT_R * 16.0/35.0);
            try:
                rc_left.SpeedM2(rc_address_left, speed_left_f);
                rc_left.SpeedM1(rc_address_left, speed_left_r);
            except:
                print "Mobile base.-> Error while writing speeds to roboclaw left"
            try:
                rc_right.SpeedM2(rc_address_right, -speed_right_f);
                rc_right.SpeedM1(rc_address_right, -speed_right_r);
            except:
                print "Mobile base.-> Error while writing speeds to roboclaw right"
        #Getting encoders for odometry calculation
        encoder_left_f  = rc_left.ReadEncM2(rc_address_left)[1];
        encoder_left_r  = rc_left.ReadEncM1(rc_address_left)[1];
        encoder_right_f = rc_right.ReadEncM2(rc_address_right)[1];
        encoder_right_r = rc_right.ReadEncM1(rc_address_right)[1];
        delta_left_f  = encoder_left_f  - encoder_last_left_f;
        delta_left_r  = encoder_left_r  - encoder_last_left_r;
        delta_right_f = encoder_right_f - encoder_last_right_f;
        delta_right_r = encoder_right_r - encoder_last_right_r;
        encoder_last_left_f  = encoder_left_f 
        encoder_last_left_r  = encoder_left_r 
        encoder_last_right_f = encoder_right_f
        encoder_last_right_r = encoder_right_r
        if abs(delta_left_f) < 24000 and abs(delta_left_r) < 24000 and abs(delta_right_f) < 24000 and abs(delta_right_r) < 24000:
            (robot_x,robot_y,robot_t) = calculate_odometry(robot_x,robot_y,robot_t,delta_left_f, delta_left_r, delta_right_f, delta_right_r);
        else:
            print "MobileBase.->Invalid encoder readings. OMFG!!!!!!!"
            print "Encoders delta: " + str(delta_left_f) + "\t" + str(delta_left_r) + "\t" + str(delta_right_f) + "\t" + str(delta_right_r);

        quaternion = tf.transformations.quaternion_from_euler(0, 0, robot_t);
        br.sendTransform((robot_x, robot_y, 0), quaternion, rospy.Time.now(), "base_link", "odom");
        pubBattery.publish(Float32(rc_left.ReadMainBatteryVoltage(rc_address_left)[1]));
        rate.sleep();

    print "MobileBase.->Stopping motors..."
    rc_left.ForwardM1(rc_address_left, 0);
    rc_left.ForwardM2(rc_address_left, 0);
    rc_right.ForwardM1(rc_address_right, 0);
    rc_right.ForwardM2(rc_address_right, 0);
        
if __name__ == '__main__':
    if "--help" in sys.argv or "-h" in sys.argv:
        print_help();
        sys.exit();
    port_left  = "/dev/ttyACM0";
    port_right = "/dev/ttyACM1";    
    if "--port1" in sys.argv:
        port_left  = sys.argv[sys.argv.index("--port1") + 1];
    if "--port2" in sys.argv:
        port_right = sys.argv[sys.argv.index("--port2") + 1];
    main(port_left, port_right);
