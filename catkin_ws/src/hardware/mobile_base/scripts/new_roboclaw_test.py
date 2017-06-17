#!/usr/bin/env python
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import roboclaw
import tf

def print_help():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callback_stop(msg):
    global new_data;
    global speed_left; 
    global speed_right;
    global speed_front;
    global speed_rear;
    speed_left  = 0;
    speed_right = 0;
    speed_front = 0;
    speed_rear  = 0;
    new_data    = True;

def check_speed_ranges(s_left, s_right, s_front, s_rear):
    max_value_frontal = 0;
    max_value_lateral = 0;
    if math.fabs(s_left)  > max_value_frontal:
        max_value_frontal = math.fabs(s_left);
    if math.fabs(s_right) > max_value_frontal:
        max_value_frontal = math.fabs(s_right);
        
    if math.fabs(s_front) > max_value_lateral:
        max_value_lateral = math.fabs(s_front);
    if math.fabs(s_rear)  > max_value_lateral:
        max_value_lateral = math.fabs(s_rear);
    #Max lateral is 1.0 and max frontal is 2.0 because motors on mobile_base are different.
    #When all motors are equal, max value will be 1.0 for all speeds
    if max_value_lateral > 1.0:
        s_left  /= max_value_lateral;
        s_right /= max_value_lateral;
        s_front /= max_value_lateral;
        s_rear  /= max_value_lateral;
    if max_value_frontal > 2.0:
        s_left  /= max_value_frontal;
        s_right /= max_value_frontal;
        s_front /= max_value_frontal;
        s_rear  /= max_value_frontal;
    return (s_left, s_right, s_front, s_rear);
    
def callback_speeds(msg):
    #Speeds are assumed to be floats in [-1,1] for each tire. The values in [0,1] need to be transformed to [0,QPPR]
    #where QPPR is the maximum motor speed. These constant is the number of encoders ticks per turn times
    #the motor angular speed. It can also be obtained with IonMotion Studio
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    global new_data;
    global speed_left; 
    global speed_right;
    global speed_front;
    global speed_rear;
    speed_left  = msg.data[0];
    speed_right = msg.data[1];
    speed_front = 0.5 * (speed_right - speed_left);
    speed_rear  = 0.5 * (speed_left - speed_right);
    
    speed_left, speed_right, speed_front, speed_rear = check_speed_ranges(speed_left, speed_right, speed_front, speed_rear);
    new_data = True;

def callback_cmd_vel(msg):
    global new_data;
    global speed_left; 
    global speed_right;
    global speed_front;
    global speed_rear;
    speed_left  = msg.linear.x - msg.angular.z * 0.48 / 2.0; #0.48 is Justina's diameter
    speed_right = msg.linear.x + msg.angular.z * 0.48 / 2.0;
    speed_front = msg.linear.y + msg.angular.z * 0.48 / 2.0;
    speed_rear  = msg.linear.y - msg.angular.z * 0.48 / 2.0;

    speed_left, speed_right, speed_front, speed_rear = check_speed_ranges(speed_left, speed_right, speed_front, speed_rear);
    new_data = True;

def calculate_odometry(pos_x, pos_y, pos_theta, enc_left, enc_right, enc_front, enc_rear):
    TICKS_PER_METER_FRONTAL = 158891.2; #Ticks per meter for the fast motors (left and right)
    TICKS_PER_METER_LATERAL = 336857.5; #Ticks per meter for the slow motors (front and rear)
    enc_left  /= TICKS_PER_METER_FRONTAL;
    enc_right /= TICKS_PER_METER_FRONTAL;
    enc_front /= TICKS_PER_METER_LATERAL;
    enc_rear  /= TICKS_PER_METER_LATERAL;

    delta_theta = (enc_right - enc_left + enc_front - enc_rear) / 0.48 / 2.0;
    if math.fabs(delta_theta) >= 0.00001:
        rg_x = (enc_left + enc_right)/(2 * delta_theta);
        rg_y = (enc_rear + enc_front)/(2 * delta_theta);
        delta_x = rg_x * math.sin(delta_theta)      + rg_y * (1-math.cos(delta_theta));
        delta_y = rg_x * (1-math.cos(delta_theta))  + rg_y * math.sin(delta_theta);
    else:
        delta_x = (enc_left + enc_right)/2.0;
        delta_y = (enc_front + enc_rear)/2.0;
    pos_x += delta_x * math.cos(pos_theta) - delta_y * math.sin(pos_theta);
    pos_y += delta_x * math.sin(pos_theta) + delta_y * math.cos(pos_theta);
    pos_theta += delta_theta;
    return (pos_x, pos_y, pos_theta);

def main(port_name_1, port_name_2):
    print "MobileBase.-> INITIALIZING THE AMAZING MOBILE BASE NODE BY MARCOSOFT..."

    #ROS CONNECTION
    rospy.init_node("omni_mobile_base");
    pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
    subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop);
    subSpeeds  = rospy.Subscriber("/hardware/mobile_base/speeds",  Float32MultiArray, callback_speeds);
    subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callback_cmd_vel);
    br   = tf.TransformBroadcaster()
    rate = rospy.Rate(30);

    #ROBOCLAW CONNECTION
    rc_frontal  = roboclaw.Roboclaw(port_name_1, 250000);
    rc_lateral  = roboclaw.Roboclaw(port_name_2, 250000);
    rc_address1 = 0x80;
    rc_address2 = 0x80;
    if rc_frontal.Open() != 1:
        print "MobileBase.-> Cannot open port " + port_name_1;
        return;
    #if rc_lateral.Open() != 1:
    #    print "MobileBase.-> Cannot open port " + port_name_2;
    #    return;
    print "MobileBase.-> Roboclaw1 open on port " + port_name_1;
    print "MobileBase.-> Roboclaw2 open on port " + port_name_2;
    rc_frontal.ResetEncoders(rc_address1);
    #rc_lateral.ResetEncoders(rc_address2);

    #ROBOCLAW CONFIGURATION CONSTANTS
    m_left_pos_PID  = rc_frontal.ReadM1PositionPID(rc_address1);
    m_right_pos_PID = rc_frontal.ReadM2PositionPID(rc_address1);
    #m_front_pos_PID = rc_lateral.ReadM1PositionPID(rc_address2);
    #m_rear_pos_PID  = rc_lateral.ReadM2PositionPID(rc_address2);
    m_left_vel_PID  = rc_frontal.ReadM1VelocityPID(rc_address1);
    m_right_vel_PID = rc_frontal.ReadM2VelocityPID(rc_address1);
    #m_front_vel_PID = rc_lateral.ReadM1VelocityPID(rc_address2);
    #m_rear_vel_PID  = rc_lateral.ReadM2VelocityPID(rc_address2);
    print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos";
    print "MobileBase.->Left Motor: "  + str(m_left_pos_PID);
    print "MobileBase.->Right Motor: " + str(m_right_pos_PID);
    #print "MobileBase.->Front Motor: " + str(m_front_pos_PID);
    #print "MobileBase.->Rear Motor: "  + str(m_rear_pos_PID);
    print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS"; #QPPS = speed in ticks/s when motor is at full speed
    print "MobileBase.->Left Motor: "  + str(m_left_vel_PID);
    print "MobileBase.->Right Motor: " + str(m_right_vel_PID);
    #print "MobileBase.->Front Motor: " + str(m_front_vel_PID);
    #print "MobileBase.->Left Motor: "  + str(m_left_vel_PID);
    QPPS_M_LEFT  = m_left_vel_PID[4];
    QPPS_M_RIGHT = m_right_vel_PID[4];
    QPPS_M_FRONT = 1000;#m_front_vel_PID[4];
    QPPS_M_REAR  = 1000;#m_rear_vel_PID[4];
    if QPPS_M_LEFT != QPPS_M_RIGHT or QPPS_M_REAR != QPPS_M_FRONT:
        print "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur."
    if rc_frontal.ReadPWMMode(rc_address1)[1] == 1:
        print "MobileBase.->PWM Mode for left and right motors: Sign magnitude"
    else:
        print "MobileBase.->PWM Mode for left and right motors: Locked antiphase"
    #if rc_lateral.ReadPWMMode(rc_address2) == 1:
    #    print "MobileBase.->PWM Mode for front and rear motors: Sign magnitude"
    #else:
    #    print "MobileBase.->PWM Mode for front and rear motors: Locked antiphase"

    #Variable initialization
    M_LR_ACCELERATION = 15000;          #Acceleration for speed control of left and right motors
    M_FR_ACCELERATION = 30000;          #Acceleration for speed control of front and rear motors
    global new_data;
    global speed_left; 
    global speed_right;
    global speed_front;
    global speed_rear;
    speed_left  = 0;    
    speed_right = 0;    
    speed_front = 0;    
    speed_rear  = 0;    
    new_data    = False;
    no_new_data_counter = 10;  #If no new speed data arrive after 5 cycles, motors are stopped.
    robot_x = 0;
    robot_y = 0;
    robot_t = 0;
    encoder_left_last  = 0;
    encoder_right_last = 0;
    encoder_front_last = 0;
    encoder_rear_last  = 0;
    
    while not rospy.is_shutdown():
        if new_data:
            new_data = False;
            no_new_data_counter = 10;
            speed_left  = int(speed_left *16.0/35.0*QPPS_M_LEFT); #Factor 16/35 is due to the different motor reductions
            speed_right = int(speed_right*16.0/35.0*QPPS_M_RIGHT); #Factor 16/35 is due to the different motor reductions
            speed_front = int(speed_front*QPPS_M_FRONT);
            speed_rear  = int(speed_rear *QPPS_M_REAR);
            try:
                rc_frontal.SpeedAccelM1M2(rc_address1, M_LR_ACCELERATION, speed_left, speed_right);
                #rc_lateral.SpeedAccelM1M2(rc_address2, M_FR_ACCELERATION, speed_front, speed_rear);
            except:
                print "MobileBase.->Error while sending speed to left-right Roboclaw";
        else:
            no_new_data_counter -= 1;
            if no_new_data_counter == 0:
                rc_frontal.ForwardM1(rc_address1, 0);
                rc_frontal.ForwardM2(rc_address1, 0);
                #rc_lateral.ForwardM1(rc_address2, 0);
                #rc_lateral.ForwardM2(rc_address2, 0);
            if no_new_data_counter < -1:
                no_new_data_counter = -1;
        encoder_left  = rc_frontal.ReadEncM1(rc_address1)[1];
        encoder_right = rc_frontal.ReadEncM2(rc_address1)[1];
        encoder_front = 0;#rc_lateral.ReadEncM1(rc_address2)[1];
        encoder_rear  = 0;#rc_lateral.ReadEncM2(rc_address2)[1];
        delta_left  = encoder_left  - encoder_left_last;
        delta_right = encoder_right - encoder_right_last;
        delta_front = encoder_front - encoder_front_last;
        delta_rear  = encoder_rear  - encoder_rear_last;
        encoder_left_last  = encoder_left;
        encoder_right_last = encoder_right;
        encoder_front_last = encoder_front;
        encoder_rear_last  = encoder_rear;
        if(math.fabs(delta_left) < 10000 and math.fabs(delta_right) < 10000
               and math.fabs(delta_front) < 10000 and math.fabs(delta_rear) < 10000):
            (robot_x,robot_y,robot_t)=calculate_odometry(robot_x,robot_y,robot_t,delta_left,delta_right,delta_front,delta_rear);
        else:
            print "MobileBase.->Invalid encoder readings.";

        quaternion = tf.transformations.quaternion_from_euler(0, 0, robot_t);
        br.sendTransform((robot_x, robot_y, 0), quaternion, rospy.Time.now(), "base_link", "odom");
        pubBattery.publish(Float32(rc_frontal.ReadMainBatteryVoltage(rc_address1)[1]));
        rate.sleep();

    print "MobileBase.->Stopping motors..."
    rc_frontal.ForwardM1(rc_address1, 0);
    rc_frontal.ForwardM2(rc_address1, 0);
    #rc_lateral.ForwardM1(rc_address2, 0);
    #rc_lateral.ForwardM2(rc_address2, 0);
    
if __name__ == '__main__':
    if "--help" in sys.argv or "-h" in sys.argv:
        print_help();
        sys.exit();
    
    port_1 = "/dev/ttyACM0";
    port_2 = "/dev/ttyACM1";
    if "--port1" in sys.argv:
        port_1 = sys.argv[sys.argv.index("--port1") + 1];
    if "--port2" in sys.argv:
        port_2 = sys.argv[sys.argv.index("--port2") + 1];
    main(port_1, port_2);
