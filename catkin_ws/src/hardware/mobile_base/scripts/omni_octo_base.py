#!/usr/bin/env python
from math import pi, cos, sin, fabs

import diagnostic_msgs
import diagnostic_updater
from hardware_tools import roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry


#import serial, time, sys, math
#import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
#from geometry_msgs.msg import TransformStamped
#from geometry_msgs.msg import Twist
#from hardware_tools import roboclaw
#import tf

class EncoderOdom:
    def __init__(self, ticks_per_meter_frontal, ticks_per_meter_lateral, base_width):
        self.TICKS_PER_METER_FRONTAL = ticks_per_meter_frontal
        self.TICKS_PER_METER_LATERAL = ticks_per_meter_lateral
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_t = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_theta = 0.0
        self.last_enc_left_f  = 0
        self.last_enc_right_r = 0
        self.last_enc_right_f = 0
        self.last_enc_left_r  = 0

        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left_f, enc_right_r, enc_right_f, enc_left_r):
        left_f_ticks  = enc_left_f  - self.last_enc_left_f
        right_r_ticks = enc_right_r - self.last_enc_right_r
        right_f_ticks = enc_right_f - self.last_enc_right_f
        left_r_ticks  = enc_left_r  - self.last_enc_left_r
        self.last_enc_left_f  = enc_left_f
        self.last_enc_right_r = enc_right_r
        self.last_enc_right_f = enc_right_f
        self.last_enc_left_r  = enc_left_r
        #print "Encoders delta: " + str(enc_left) + "\t" + str(enc_right) + "\t" +  str(enc_front) + "\t" + str(enc_rear);
        
        dist_left_f  = left_f_ticks  / self.TICKS_PER_METER_FRONTAL
        dist_right_r = right_r_ticks / self.TICKS_PER_METER_FRONTAL
        dist_right_f = right_f_ticks / self.TICKS_PER_METER_LATERAL
        dist_left_r  = left_r_ticks  / self.TICKS_PER_METER_LATERAL
        
        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time
 
        #dist_x = (dist_left_f + dist_right_f + dist_left_r + dist_right_r) / 4.0
        #dist_y = (dist_right_f + dist_left_r - dist_right_r - dist_left_f) / 4.0 * 0.707

        #TODO CHeck if is correct the odometry
        delta_theta = (dist_right_f - dist_left_f + dist_right_r - dist_left_r)/self.BASE_WIDTH / 2.0
        if fabs(delta_theta) >= 0.00001:
            #rg_x = (dist_left_f + dist_right_f + dist_left_r + dist_right_r)/(4.0 * delta_theta);
            #rg_y = (dist_right_f + dist_left_r - dist_right_r - dist_left_f)/(4.0 * delta_theta) * 0.707;
            rg_x = (dist_right_r + dist_left_f)/(2*delta_theta)
            rg_y = (dist_right_f + dist_left_r)/(2*delta_theta)
            delta_x = rg_x * sin(delta_theta)       + rg_y * (1 - cos(delta_theta))
            delta_y = rg_x * (1 - cos(delta_theta)) + rg_y * sin(delta_theta)    
        else: 
            #delta_x = dist_x
            #delta_y = dist_y
            delta_x = (dist_right_r + dist_left_f)/2.0
            delta_y = (dist_right_f + dist_left_r)/2.0
        dist_x =  0.707106781*delta_x + 0.707106781*delta_y
        dist_y = -0.707106781*delta_x + 0.707106781*delta_y
        self.robot_x += dist_x * cos(self.robot_t) - dist_y * sin(self.robot_t)
        self.robot_y += dist_x * sin(self.robot_t) + dist_y * cos(self.robot_t)
        self.robot_t  = self.normalize_angle(self.robot_t + delta_theta)

        
        
        #vel_y = 0.0
        #if left_ticks == right_ticks:
        #    delta_theta = 0.0
        #    self.robot_x += dist_x * cos(self.robot_t)
        #    self.robot_y += dist_x * sin(self.robot_t)
        #else:
        #    delta_theta = (dist_right - dist_left) / self.BASE_WIDTH
        #    r = dist_x / delta_theta
        #    self.robot_x += r * (sin(delta_theta + self.robot_t) - sin(self.robot_t))
        #    self.robot_y -= r * (cos(delta_theta + self.robot_t) - cos(self.robot_t))
        #    self.robot_t = self.normalize_angle(self.robot_t + delta_theta)

        if abs(d_time) < 0.000001:
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_theta = 0.0
        else:
            self.vel_x = dist_x / d_time
            self.vel_y = dist_y / d_time
            self.vel_theta = delta_theta / d_time

    def update_publish(self, enc_left_f, enc_right_r, enc_right_f, enc_left_r):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        # TODO Check the range of the delta encoder from the lateral motors
        #if abs(enc_left - self.last_enc_left) < 24000 and abs(enc_right - self.last_enc_right) < 24000 and abs(enc_front - self.last_enc_front) < 48000 and abs(enc_rear - self.last_enc_rear) < 48000:
        #if abs(enc_left - self.last_enc_left) < 48000 and abs(enc_right - self.last_enc_right) < 48000 and abs(enc_front - self.last_enc_front) < 48000 and abs(enc_rear - self.last_enc_rear) < 48000:
        if abs(enc_left_f - self.last_enc_left_f) < 93000 and abs(enc_right_r - self.last_enc_right_r) < 93000 and abs(enc_right_f - self.last_enc_right_f) < 93000 and abs(enc_left_r - self.last_enc_left_r) < 93000:
            self.update(enc_left_f, enc_right_r, enc_right_f, enc_left_r)
        else:
            rospy.logerr("MobileBase.->Invalid encoder readings. OMFG!!!!!!!")
            rospy.logerr("Ignoring left front encoder jump: cur %d, last %d"  % (enc_left_f,  self.last_enc_left_f ))
            rospy.logerr("Ignoring right rear encoder jump: cur %d, last %d" % (enc_right_r, self.last_enc_right_r))
            rospy.logerr("Ignoring right front encoder jump: cur %d, last %d" % (enc_right_f, self.last_enc_right_f))
            rospy.logerr("Ignoring left rear encoder jump: cur %d, last %d"  % (enc_left_r,  self.last_enc_left_r ))
            self.last_enc_left_f  = enc_left_f
            self.last_enc_right_r = enc_right_r
            self.last_enc_right_f = enc_right_f
            self.last_enc_left_r  = enc_left_r
        self.publish_odom(self.robot_x, self.robot_y, self.robot_t, self.vel_x, self.vel_y, self.vel_theta)

    def publish_odom(self, robot_x, robot_y, robot_t, vx, vy, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, robot_t)
        current_time = rospy.Time.now()
        br = tf.TransformBroadcaster()
        #TODO Check if is working or change to the old node
        br.sendTransform((robot_x, robot_y, 0), 
                quat, 
                current_time, 
                "base_link", 
                "odom")

        #TODO Check this parametters to the odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        
        odom.pose.pose.position.x = robot_x
        odom.pose.pose.position.y = robot_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)
        
class MobileOmniBaseNode:
    def __init__(self):
        print "MobileBase.-> INITIALIZING THE AMAZING MOBILE BASE NODE BY REY..."
        rospy.init_node("mobile_base");
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Trying to Connect to roboclaws")

        port_name_frontal = "/dev/ttyACM0";
        port_name_lateral = "/dev/ttyACM1";
        
        self.simul = False
        self.ControlP = 1.0
        self.ControlI = 1.0
        self.ControlD = 1.0
    
        if rospy.has_param('~simul'):
            self.simul = rospy.get_param('~simul')
        if rospy.has_param('~port1'):
            port_name_frontal = rospy.get_param('~port1')
        elif not self.simul:
            print_help();
            sys.exit();
        if rospy.has_param('~port2'):
            port_name_lateral = rospy.get_param('~port2')
        elif not self.simul:
            #TODO put the correct config ros param help
            print_help();
            sys.exit();
        if rospy.has_param('~ControlP'):
            self.ControlP = int(rospy.get_param('~ControlP'))
        if rospy.has_param('~ControlI'):
            self.ControlI = int(rospy.get_param('~ControlI'))
        if rospy.has_param('~ControlD'):
            self.ControlD = int(rospy.get_param('~ControlD'))
            
        
        #TODO If is necessary add the ros param address roboclaw
        self.rc_address_frontal = 0x80
        self.rc_address_lateral = 0x80
        #self.BASE_WIDTH = 0.6
        self.BASE_WIDTH = 0.46
        #self.TICKS_PER_METER_LATERAL = 103072.0 #Ticks per meter for the slow motors (front and rear)
        #self.TICKS_PER_METER_FRONTAL = 103072.0 #Ticks per meter for the fast motors (left and right)
        self.TICKS_PER_METER_LATERAL = 139071.0 #Ticks per meter for the slow motors (front and rear)
        self.TICKS_PER_METER_FRONTAL = 139071.0 #Ticks per meter for the fast motors (left and right)
        
        #baud_rate_frontal = 38400
        #baud_rate_lateral = 38400
        baud_rate_frontal = 115200
        baud_rate_lateral = 115200

        self.speed_left_f  = 0
        self.speed_right_r = 0
        self.speed_right_f = 0
        self.speed_left_r  = 0

        if not self.simul:
            self.rc_frontal = roboclaw.Roboclaw(port_name_frontal, baud_rate_frontal); #Roboclaw controling motors for frontal movement (left and right)
            self.rc_lateral = roboclaw.Roboclaw(port_name_lateral, baud_rate_lateral); #Roboclaw controling motors for lateral movement (front and rear)

            if self.rc_address_frontal > 0x87 or self.rc_address_frontal < 0x80:
                rospy.logfatal("Address frontal out of range")
                rospy.signal_shutdown("Address lateral out of range")
            if self.rc_address_frontal > 0x87 or self.rc_address_frontal < 0x80:
                rospy.logfatal("Address frontal out of range")
                rospy.signal_shutdown("Address lateral out of range")
        
            try:
                self.rc_frontal.Open()
            except Exception as e:
                rospy.logfatal("Could not connect to frontal Roboclaw")
                rospy.logdebug(e)
                rospy.signal_shutdown("Could not connect to frontal Roboclaw")
            try:
                self.rc_lateral.Open()
            except Exception as e:
                rospy.logfatal("Could not connect to lateral Roboclaw")
                rospy.logdebug(e)
                rospy.signal_shutdown("Could not connect to lateral Roboclaw")

            #TODO Config the diagnostic config

            try:
                frontal_version = self.rc_frontal.ReadVersion(self.rc_address_frontal)
            except Exception as e:
                rospy.logwarn("Problem getting roboclaw frontal version")
                rospy.logdebug(e)
                pass

            if not frontal_version[0]:
                rospy.logwarn("Could not get version from frontal roboclaw")
            else:
                rospy.logdebug(repr(frontal_version[1]))
            
            try:
                lateral_version = self.rc_frontal.ReadVersion(self.rc_address_lateral)
            except Exception as e:
                rospy.logwarn("Problem getting roboclaw lateral version")
                rospy.logdebug(e)
                pass

            if not lateral_version[0]:
                rospy.logwarn("Could not get version from lateral roboclaw")
            else:
                rospy.logdebug(repr(lateral_version[1]))

            self.rc_frontal.SpeedM1M2(self.rc_address_frontal, 0, 0)
            self.rc_frontal.ResetEncoders(self.rc_address_frontal)
            self.rc_lateral.SpeedM1M2(self.rc_address_lateral, 0, 0)
            self.rc_lateral.ResetEncoders(self.rc_address_lateral)
            #ROBOCLAW CONFIGURATION CONSTANT
            print "MobileBase.->Control PID constants for Velocity passed by parameter:"
            print "MobileBase.->P = " + str(self.ControlP)
            print "MobileBase.->I = " + str(self.ControlI)
            print "MobileBase.->D = " + str(self.ControlD)
            #pos_PID_left  = self.rc_frontal.ReadM1PositionPID(self.rc_address_frontal)
            #pos_PID_right = self.rc_frontal.ReadM2PositionPID(self.rc_address_frontal)
            #pos_PID_front = self.rc_lateral.ReadM1PositionPID(self.rc_address_lateral)
            #pos_PID_rear  = self.rc_lateral.ReadM2PositionPID(self.rc_address_lateral)
            vel_PID_left  = self.rc_frontal.ReadM1VelocityPID(self.rc_address_frontal)
            vel_PID_right = self.rc_frontal.ReadM2VelocityPID(self.rc_address_frontal)
            vel_PID_front = self.rc_lateral.ReadM1VelocityPID(self.rc_address_lateral)
            vel_PID_rear  = self.rc_lateral.ReadM2VelocityPID(self.rc_address_lateral)
            #print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos"
            #print "MobileBase.->Left Motor:  " + str(pos_PID_left )
            #print "MobileBase.->Right Motor: " + str(pos_PID_right)
            #print "MobileBase.->Front Motor: " + str(pos_PID_front)
            #print "MobileBase.->Rear Motor:  " + str(pos_PID_rear )
            print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS" #QPPS = speed in ticks/s when motor is at full speed
            print "MobileBase.->Left Motor:  " + str(vel_PID_left )
            print "MobileBase.->Right Motor: " + str(vel_PID_right)
            print "MobileBase.->Front Motor: " + str(vel_PID_front)
            print "MobileBase.->Left Motor:  " + str(vel_PID_rear )
            self.QPPS_LEFT_F  = vel_PID_left[4]
            self.QPPS_RIGHT_R = vel_PID_right[4]
            self.QPPS_RIGHT_F = vel_PID_front[4]
            self.QPPS_LEFT_R  = vel_PID_rear[4]
            if self.QPPS_LEFT_F != self.QPPS_RIGHT_R or self.QPPS_RIGHT_F != self.QPPS_LEFT_R:
                print "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur."
            if self.rc_frontal.ReadPWMMode(self.rc_address_frontal)[1] == 1:
                print "MobileBase.->PWM Mode for left and right motors: Sign magnitude"
            else:
                print "MobileBase.->PWM Mode for left and right motors: Locked antiphase"
            if self.rc_lateral.ReadPWMMode(self.rc_address_lateral)[1] == 1:
                print "MobileBase.->PWM Mode for front and rear motors: Sign magnitude"
            else:
                print "MobileBase.->PWM Mode for front and rear motors: Locked antiphase"

        else:
            self.QPPS_LEFT_F  = self.TICKS_PER_METER_FRONTAL
            self.QPPS_RIGHT_R = self.TICKS_PER_METER_FRONTAL
            self.QPPS_RIGHT_F = self.TICKS_PER_METER_LATERAL
            self.QPPS_LEFT_R  = self.TICKS_PER_METER_LATERAL
        
        self.encodm = EncoderOdom(self.TICKS_PER_METER_FRONTAL, self.TICKS_PER_METER_LATERAL, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()
        rospy.sleep(1)
        
        self.pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
        self.pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
        #subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop, queue_size=1);
        self.subSpeeds  = rospy.Subscriber("/hardware/mobile_base/speeds",  Float32MultiArray, self.cmd_speeds_callback, queue_size=1);
        self.subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1);
        self.subSimul   = rospy.Subscriber("/simulated", Bool, self.callback_simulated, queue_size = 1);

    def run(self):
        rate = rospy.Rate(30)
        encoder_left_f  = 0
        encoder_right_r = 0
        encoder_right_f = 0
        encoder_left_r  = 0
        self.newData = False
        self.no_new_data_counter = 5
        jointStates = JointState()
        jointStates.name = ["wheel_front_left_connect", "wheel_back_right_connect", "wheel_front_right_connect", "wheel_back_left_connect"]
        jointStates.position = [0 , 0, 0, 0]
        while not rospy.is_shutdown():
            if not self.simul:
                if self.newData:
                    self.newData = False
                    
                    #self.speed_left_f  =  int(self.speed_left_f  * self.QPPS_LEFT_F  * 24.0/35.0)
                    #self.speed_right_r = -int(self.speed_right_r * self.QPPS_RIGHT_R * 24.0/35.0)
                    #self.speed_right_f = -int(self.speed_right_f * self.QPPS_RIGHT_F * 24.0/35.0)                                           
                    #self.speed_left_r  = int(self.speed_left_r  * self.QPPS_LEFT_R  * 24.0/35.0)
                    self.speed_left_f  =  int(self.speed_left_f  * self.TICKS_PER_METER_FRONTAL)
                    self.speed_right_r = -int(self.speed_right_r * self.TICKS_PER_METER_FRONTAL)
                    self.speed_right_f = -int(self.speed_right_f * self.TICKS_PER_METER_LATERAL)                                           
                    self.speed_left_r  =  int(self.speed_left_r  * self.TICKS_PER_METER_LATERAL)
            
                    try:
                        # This is a hack way to keep a poorly tuned PID from making noise at speed 0
                        #if self.speed_left is 0 and self.speed_right is 0:
                        #    self.rc_frontal.ForwardM1(self.rc_address_frontal, 0)
                        #    self.rc_frontal.ForwardM2(self.rc_address_frontal, 0)
                        #else:
                        #self.rc_frontal.SpeedM1M2(self.rc_address_frontal, self.speed_left_f, self.speed_right_f)
                        print
                    except OSError as e:
                        rospy.logwarn("SpeedM1M2 frontal OSError: %d", e.errno)
                        rospy.logdebug(e)
                    except Exception as e:
                        rospy.logerr("Mobile base.-> Error while writing speeds to roboclaw frontal")
                        rospy.logdebug(e)
                    try:
                        # This is a hack way to keep a poorly tuned PID from making noise at speed 0
                        #if self.speed_front is 0 and self.speed_rear is 0:
                        #    self.rc_lateral.ForwardM1(self.rc_address_lateral, 0)
                        #    self.rc_lateral.ForwardM2(self.rc_address_lateral, 0)
                        #else:
                        self.rc_lateral.SpeedM1M2(self.rc_address_lateral, self.speed_left_r, self.speed_right_r)
                    except OSError as e:
                        rospy.logwarn("SpeedM1M2 lateral OSError: %d", e.errno)
                        rospy.logdebug(e)
                    except Exception as e:
                        rospy.logerr("Mobile base.-> Error while writing speeds to roboclaw lateral")
                        rospy.logdebug(e)
                else:
                    if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 0.3:
                        self.no_new_data_counter -= 1;
                        if self.no_new_data_counter == 0:
                            #rospy.loginfo("Did not get command for 1 second, stopping")
                            try:
                                self.rc_frontal.ForwardM1(self.rc_address_frontal, 0)
                                self.rc_frontal.ForwardM2(self.rc_address_frontal, 0)
                            except OSError as e:
                                rospy.logerr("Could not stop")
                                rospy.logdebug(e)
                            try:
                                self.rc_lateral.ForwardM1(self.rc_address_lateral, 0)
                                self.rc_lateral.ForwardM2(self.rc_address_lateral, 0)
                            except OSError as e:
                                rospy.logerr("Could not stop")
                                rospy.logdebug(e)
                        if self.no_new_data_counter < -1:
                            self.no_new_data_counter = -1;
            
                try:
                    status_left_f, encoder_left_f, crc_left_f = self.rc_frontal.ReadEncM1(self.rc_address_frontal)
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 frontal OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_right_f, encoder_right_f, crc_right_f = self.rc_frontal.ReadEncM2(self.rc_address_frontal)
                    encoder_right_f = -encoder_right_f
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM2 frontal OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_left_r, encoder_left_r, crc_left_r = self.rc_lateral.ReadEncM1(self.rc_address_lateral)
                    encoder_left_r = -encoder_left_r
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 lateral OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_right_r, encoder_right_r, crc_right_r = self.rc_lateral.ReadEncM2(self.rc_address_lateral)
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM2 lateral OSError: %d", e.errno)
                    rospy.logdebug(e)
            else:
                if not self.newData:
                    self.no_new_data_counter -= 1;
                    if self.no_new_data_counter == 0:
                        self.speed_left_f = 0
                        self.speed_right_r = 0
                        self.speed_right_f = 0
                        self.speed_left_r = 0
                    if self.no_new_data_counter < -1:
                        self.no_new_data_counter = -1;
                else:
                    self.newData = False
                encoder_left_f  = encoder_left_f  + self.speed_left_f   * 0.05 * self.QPPS_LEFT_F
                encoder_right_r = encoder_right_r + self.speed_right_r  * 0.05 * self.QPPS_RIGHT_R
                encoder_right_f = encoder_right_f + self.speed_right_f  * 0.05 * self.QPPS_RIGHT_F
                encoder_left_r  = encoder_left_r  + self.speed_left_r   * 0.05 * self.QPPS_LEFT_R

            #try:
            #    if not self.simul:
            #        #self.pubBattery.publish(Float32(self.rc_frontal.ReadMainBatteryVoltage(self.rc_address_frontal)[1]))
            #    else:
            #        self.pubBattery.publish(Float32(12.0));
            #except OSError as e:
            #    rospy.logwarn("ReadMainBatteryVoltage Frontal OSError: %d", e.errno)
            #    rospy.logdebug(e)
            #except Exception as e:
            #    rospy.logdebug(e)
            #TODO Update the updater function
            self.encodm.update_publish(encoder_left_f, encoder_right_r, encoder_right_f, encoder_left_r)
            jointStates.header.stamp = rospy.Time.now()
            jointStates.position[0] = 0
            jointStates.position[1] = 0
            jointStates.position[2] = 0
            jointStates.position[3] = 0
            self.pubJointStates.publish(jointStates)
            rate.sleep();

    def shutdown(self):
        rospy.loginfo("Shutting down")
        if not self.simul:
            try:
                self.rc_frontal.ForwardM1(self.rc_address_frontal, 0)
                self.rc_frontal.ForwardM2(self.rc_address_frontal, 0)
            except OSError:
                rospy.logerr("Shutdown did not work the frontal motors trying again")
                try:
                    self.rc_frontal.ForwardM1(self.rc_address_frontal, 0)
                    self.rc_frontal.ForwardM2(self.rc_address_frontal, 0)
                except OSError as e:
                    rospy.logerr("Could not shutdown the frontal motors!!!!")
                    rospy.logdebug(e)
            try:
                self.rc_lateral.ForwardM1(self.rc_address_lateral, 0)
                self.rc_lateral.ForwardM2(self.rc_address_lateral, 0)
            except OSError:
                rospy.logerr("Shutdown did not work the frontal motors trying again")
                try:
                    self.rc_lateral.ForwardM1(self.rc_lateral_frontal, 0)
                    self.rc_lateral.ForwardM2(self.rc_lateral_frontal, 0)
                except OSError as e:
                    rospy.logerr("Could not shutdown the lateral motors!!!!")
                    rospy.logdebug(e)

    def cmd_speeds_callback(self, msg):
        print "MOBILE BASE.-> WARNING!!! THIS FUNCTION IS DEPRECATED!! USE cmd_vel INSTEAD"
        #self.speed_left_f  =  msg.data[0];
        #self.speed_right_r =  msg.data[0];
        #self.speed_right_f =  msg.data[1];
        #self.speed_left_r  =  msg.data[1];
        self.speed_left_f  = 0
        self.speed_right_r = 0
        self.speed_right_f = 0
        self.speed_left_r  = 0
        self.newData = True
        self.no_new_data_counter = 5;
    
    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = rospy.get_rostime()
        linear_x = 0.707106781 * twist.linear.x - 0.707106781 * twist.linear.y;
        linear_y = 0.707106781 * twist.linear.x + 0.707106781 * twist.linear.y;
        angular_z = twist.angular.z
        self.speed_left_f  = linear_x - twist.angular.z * self.BASE_WIDTH/2.0
        self.speed_right_r = linear_x + twist.angular.z * self.BASE_WIDTH/2.0
        self.speed_right_f = linear_y + twist.angular.z * self.BASE_WIDTH/2.0
        self.speed_left_r  = linear_y - twist.angular.z * self.BASE_WIDTH/2.0

        (self.speed_left_f, self.speed_right_r, self.speed_right_f, self.speed_left_r) = self.check_speed_ranges(self.speed_left_f, self.speed_right_r, self.speed_right_f, self.speed_left_r)
        self.newData = True
        self.no_new_data_counter = 5;
        #if not self.simul:

    def check_speed_ranges(self, s_left_f, s_right_r, s_right_f, s_left_r): #speeds: left, right, front and rear
        max_value = max(abs(s_left_f), abs(s_right_r), abs(s_right_f), abs(s_left_r));

        #TODO Check the max value to the frontal motors and pass the parametters how ros params
        if max_value > 2.0:
            s_left_f  = s_left_f  / max_value * 2.0;
            s_right_r = s_right_r / max_value * 2.0;
            s_right_f = s_right_f / max_value * 2.0;
            s_left_r  = s_left_r  / max_value * 2.0;
        return (s_left_f, s_right_r, s_right_f, s_left_r);

    def callback_simulated(self, msg):
        self.simul = msg.data;

if __name__ == "__main__":
    try:
        mobileOmniBaseNode = MobileOmniBaseNode()
        mobileOmniBaseNode.run()
    except rospy.ROSInterruptException:
        pass
        rospy.loginfo("Exiting")
