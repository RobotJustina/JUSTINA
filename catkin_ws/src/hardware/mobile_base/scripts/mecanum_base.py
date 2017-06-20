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

base_diameter = 0.48;
port_name_1 = "/dev/ttyACM0";
port_name_2 = "/dev/ttyACM1";
rc_address_front = 0x80;
rc_address_rear  = 0x80; 
rc_front = roboclaw.Roboclaw(port_name_1, 250000);
rc_rear  = roboclaw.Roboclaw(port_name_2, 250000);

def print_help():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callback_stop(msg):
    rc_front.ForwardM1(rc_address_front, 0);
    rc_front.ForwardM2(rc_address_front, 0);
    rc_rear.ForwardM1(rc_address_rear, 0);
    rc_rear.ForwardM2(rc_address_rear, 0);
    global new_data;
    new_data = True;

def check_speed_ranges(s_fl, s_fr, s_rl, s_rr): #speeds: front left, front right, rear left, rear right
    max_value = 0;
    if math.fabs(s_fl) > max_value:
        max_value = math.fabs(s_fl);
    if math.fabs(s_fr) > max_value:
        max_value = math.fabs(s_fr);
    if math.fabs(s_rl) > max_value:
        max_value = math.fabs(s_rl);
    if math.fabs(s_rr) > max_value:
        max_value = math.fabs(s_rr);

    if max_value > 1.0:
        print "MobileBase.->Warning! Speeds should not be greater than 1.0. Normalized speeds used instead"
        s_fl /= max_value;
        s_fr /= max_value;
        s_rl /= max_value;
        s_rr /= max_value;
    return (s_fl, s_fr, s_rl, s_rr);

def callback_speeds(msg):
    #Speeds are assumed to be floats in [-1,1] for each tire. The values in [0,1] need to be transformed to [0,QPPR]
    #where QPPR is the maximum motor speed. These constant is the number of encoders ticks per turn times
    #the motor angular speed. It can also be obtained with IonMotion Studio
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    speed_fl = msg.data[0];
    speed_fr = msg.data[1];
    speed_rl = msg.data[0];
    speed_rr = msg.data[1];
    (speed_fl, speed_fr, speed_rl, speed_rr) = check_speed_ranges(speed_fl, speed_fr, speed_rl, speed_rr);
    speed_fl = int(speed_fl*32767);
    speed_fr = int(speed_fr*32767);
    speed_rl = int(speed_rl*32767 * 16.0/35.0);
    speed_rr = int(speed_rr*32767 * 16.0/35.0);
    rc_front.DutyM1M2(rc_address_front, speed_fl, speed_fr);
    rc_rear.DutyM1M2(rc_address_rear,   speed_rl, speed_rr);
    global new_data;
    new_data = True;

def callback_cmd_vel(msg):
    speed_fl = msg.linear.x - msg.angular.z*base_diameter/2.0 - msg.linear.y/2.0;
    speed_fr = msg.linear.x + msg.angular.z*base_diameter/2.0 - msg.linear.y/2.0;
    speed_rl = msg.linear.x - msg.angular.z*base_diameter/2.0 + msg.linear.y/2.0;
    speed_rr = msg.linear.x + msg.angular.z*base_diameter/2.0 + msg.linear.y/2.0;
    (speed_fl, speed_fr, speed_rl, speed_rr) = check_speed_ranges(speed_fl, speed_fr, speed_rl, speed_rr);
    speed_fl = int(speed_fl*32767);
    speed_fr = int(speed_fr*32767);
    speed_rl = int(speed_rl*32767 * 16.0/35.0);
    speed_rr = int(speed_rr*32767 * 16.0/35.0);
    rc_front.DutyM1M2(rc_address_front, speed_fl, speed_fr);
    rc_rear.DutyM1M2(rc_address_rear,   speed_rl, speed_rr);
    global newData;
    newData = True;

def calculate_odometry(pos_x, pos_y, pos_theta, enc_fl, enc_fr, enc_rl, enc_rr):
    TICKS_PER_METER_FRONT = 336857.5; #Ticks per meter for the slow motors (front and rear)
    TICKS_PER_METER_REAR  = 158891.2; #Ticks per meter for the fast motors (left and right)
    enc_fl /= TICKS_PER_METER_FRONTAL;
    enc_fr /= TICKS_PER_METER_FRONTAL;
    enc_rl /= TICKS_PER_METER_LATERAL;
    enc_rr /= TICKS_PER_METER_LATERAL;

    delta_theta = (enc_fr - enc_fl + enc_rr - enc_rl) / 0.48 / 2.0;
    if math.fabs(delta_theta) >= 0.00001:
        rg_x = (enc_fl + enc_fr + enc_rl + enc_rr)/(4.0 * delta_theta);
        rg_y = (enc_rr + enc_rl - enc_fr - enc_fl)/(4.0 * delta_theta);
        delta_x = rg_x * math.sin(delta_theta)      + rg_y * (1-math.cos(delta_theta));
        delta_y = rg_x * (1-math.cos(delta_theta))  + rg_y * math.sin(delta_theta);
    else:
        delta_x = (enc_fl + enc_fr + enc_rl + enc_rr)/4.0;
        delta_y = (enc_rr + enc_rl - enc_fr - enc_fl)/4.0;
    pos_x += delta_x * math.cos(pos_theta) - delta_y * math.sin(pos_theta);
    pos_y += delta_x * math.sin(pos_theta) + delta_y * math.cos(pos_theta);
    pos_theta += delta_theta;
    return (pos_x, pos_y, pos_theta);

def main():
    print "MobileBase.-> INITIALIZING THE AMAZING MOBILE BASE NODE BY MARCOSOFT..."

    #ROS CONNECTION
    rospy.init_node("mobile_base");
    pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
    subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop);
    subSpeeds  = rospy.Subscriber("/hardware/mobile_base/speeds",  Float32MultiArray, callback_speeds);
    subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callback_cmd_vel);
    br   = tf.TransformBroadcaster()
    rate = rospy.Rate(30);

    #ROBOCLAW CONNECTION
    if rc_front.Open() != 1:
        print "MobileBase.-> Cannot open port " + port_name_1;
        return;
    if rc_rear.Open() != 1:
        print "MobileBase.-> Cannot open port " + port_name_2;
        return;
    print "MobileBase.-> Roboclaw1 open on port " + port_name_1;
    print "MobileBase.-> Roboclaw2 open on port " + port_name_2;
    rc_front.ResetEncoders(rc_address_front);
    rc_rear.ResetEncoders(rc_address_rear);
    
    global new_data;
    new_data = False;
    no_new_data_counter = 10;  #If no new speed data arrive after 5 cycles, motors are stopped.
    robot_x = 0;
    robot_y = 0;
    robot_t = 0;
    encoder_front_left  = 0;
    encoder_front_right = 0;
    encoder_rear_left   = 0;
    encoder_rear_right  = 0;
    encoder_front_left_last  = 0;
    encoder_front_right_last = 0;
    encoder_rear_left_last   = 0;
    encoder_rear_right_last  = 0;

    while not rospy.is_shutdown():
        if not new_data:
            no_new_data_counter -= 1;
            if no_new_data_counter == 0:
                rc_front.ForwardM1(rc_address_front, 0);
                rc_front.ForwardM2(rc_address_front, 0);
                rc_rear.ForwardM1(rc_address_rear, 0);
                rc_rear.ForwardM2(rc_address_rear, 0);  
            if no_new_data_counter < -1:
                no_new_data_counter = -1;
        else:
            new_data = False;
        #Getting encoders for odometry calculation
        encoder_front_left  = rc_front.ReadEncM1(rc_address_front)[1];
        encoder_front_right = rc_front.ReadEncM2(rc_address_front)[1];
        encoder_rear_left   = rc_rear.ReadEncM1(rc_address_rear)[1];
        encoder_rear_right  = rc_rear.ReadEncM2(rc_address_rear)[1];
        delta_fl = encoder_front_left  - encoder_front_left_last; 
        delta_fr = encoder_front_right - encoder_front_right_last;
        delta_rl = encoder_rear_left   - encoder_rear_left_last;  
        delta_rr = encoder_rear_right  - encoder_rear_right_last; 
        encoder_front_left_last  = encoder_front_left; 
        encoder_front_right_last = encoder_front_right;
        encoder_rear_left_last   = encoder_rear_left;  
        encoder_rear_right_last  = encoder_rear_right; 
        if(math.fabs(delta_fl) < 10000 and math.fabs(delta_fr) < 10000
               and math.fabs(delta_rl) < 10000 and math.fabs(delta_rr) < 10000):
            (robot_x, robot_y, robot_t) = calculate_odometry(robot_x,robot_y,robot_t,delta_fl,delta_fr,delta_rl,delta_rr);
        else:
            print "MobileBase.->Invalid encoder readings.";

        quaternion = tf.transformations.quaternion_from_euler(0, 0, robot_t);
        br.sendTransform((robot_x, robot_y, 0), quaternion, rospy.Time.now(), "base_link", "odom");
        pubBattery.publish(Float32(rc_front.ReadMainBatteryVoltage(rc_address1)[1]));
        rate.sleep();

    print "MobileBase.->Stopping motors..."
    rc_front.ForwardM1(rc_address_front, 0);
    rc_front.ForwardM2(rc_address_front, 0);
    rc_rear.ForwardM1(rc_address_rear, 0);  
    rc_rear.ForwardM2(rc_address_rear, 0);  
    
if __name__ == '__main__':
    if "--help" in sys.argv or "-h" in sys.argv:
        print_help();
        sys.exit();
    global port_name_1;
    global port_name_2;    
    if "--port1" in sys.argv:
        port_name_1 = sys.argv[sys.argv.index("--port1") + 1];
    if "--port2" in sys.argv:
        port_name_2 = sys.argv[sys.argv.index("--port2") + 1];
    main();
