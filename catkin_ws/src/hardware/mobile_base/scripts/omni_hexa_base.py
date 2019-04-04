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
        self.last_enc_front_left  = 0
        self.last_enc_front_right = 0
        self.last_enc_rear = 0

        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_front_left, enc_front_right, enc_rear):
        front_left_ticks    = enc_front_left - self.last_enc_front_left
        front_right_ticks   = enc_front_right - self.last_enc_front_right
        rear_ticks          = enc_rear - self.last_enc_rear
        self.last_enc_front_left    = enc_front_left
        self.last_enc_front_right   = enc_front_right
        self.last_enc_rear          = enc_rear
        
        dist_front_left     = front_left_ticks  / self.TICKS_PER_METER_FRONTAL
        dist_front_right    = front_right_ticks / self.TICKS_PER_METER_FRONTAL
        dist_rear           = rear_ticks        / self.TICKS_PER_METER_LATERAL
        
        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time
             
        #TODO CHeck if is correct the odometry
        #delta_theta = (dist_front_right + dist_front_left + dist_rear) / self.BASE_WIDTH / 2.0
        #if fabs(delta_theta) >= 0.00001:
        #    rg_x = (dist_front_right - dist_front_left)/ (2.0 * delta_theta)
        #    rg_y = (dist_front_right + dist_front_left - 2.0 * dist_rear)/ (2.0 * delta_theta)
        #    delta_x = rg_x * sin(delta_theta)       + rg_y * (1 - cos(delta_theta))
        #    delta_y = rg_x * (1 - cos(delta_theta)) + rg_y * sin(delta_theta)    
        #else: 
        #    delta_x = (dist_front_right - dist_front_left) / 3.0
        #    delta_y = (dist_front_right + dist_front_left) / 3.0  - 2.0 * dist_rear / 3.0
        #dist_x = 1.732050808 *  delta_x
        #dist_y = delta_y
        #self.robot_x += dist_x * cos(self.robot_t) - dist_y * sin(self.robot_t)
        #self.robot_y += dist_x * sin(self.robot_t) + dist_y * cos(self.robot_t)
        #self.robot_t  = self.normalize_angle(self.robot_t + delta_theta)

        delta_theta = (dist_front_right + dist_front_left + dist_rear) / (self.BASE_WIDTH / 2.0) / 3.0
        dist_x = (1.732050808 * dist_front_right - 1.732050808 * dist_front_left) / 3.0
        dist_y = (dist_front_right + dist_front_left - 2.0 * dist_rear) / 3.0
        self.robot_t  = self.normalize_angle(self.robot_t + delta_theta)
        self.robot_x += dist_x * cos(self.robot_t) - dist_y * sin(self.robot_t)
        self.robot_y += dist_x * sin(self.robot_t) + dist_y * cos(self.robot_t)

        if abs(d_time) < 0.000001:
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_theta = 0.0
        else:
            self.vel_x = dist_x / d_time
            self.vel_y = dist_y / d_time
            self.vel_theta = delta_theta / d_time

    def update_publish(self, enc_front_left, enc_front_right, enc_rear):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        # TODO Check the range of the delta encoder from the lateral motors
        #if abs(enc_left - self.last_enc_left) < 24000 and abs(enc_right - self.last_enc_right) < 24000 and abs(enc_front - self.last_enc_front) < 48000 and abs(enc_rear - self.last_enc_rear) < 48000:
        #if abs(enc_left - self.last_enc_left) < 48000 and abs(enc_right - self.last_enc_right) < 48000 and abs(enc_front - self.last_enc_front) < 48000 and abs(enc_rear - self.last_enc_rear) < 48000:
        if abs(enc_front_left - self.last_enc_front_left) < 93000 and abs(enc_front_right - self.last_enc_front_right) < 93000 and abs(enc_rear - self.last_enc_rear) < 93000:
            self.update(enc_front_left, enc_front_right, enc_rear)
        else:
            rospy.logerr("MobileBase.->Invalid encoder readings. OMFG!!!!!!!")
            rospy.logerr("Ignoring front left encoder jump: cur %d, last %d"  % (enc_front_left,  self.last_enc_front_left ))
            rospy.logerr("Ignoring front right encoder jump: cur %d, last %d" % (enc_front_right, self.last_enc_front_right))
            rospy.logerr("Ignoring rear encoder jump: cur %d, last %d" % (enc_rear, self.last_enc_rear))
            self.last_enc_front_left    = enc_front_left
            self.last_enc_front_right   = enc_front_right
            self.last_enc_rear          = enc_rear
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
        
        #TODO If is necessary add the ros param address roboclaw
        self.rc_address_frontal = 0x80
        self.rc_address_lateral = 0x80
        #self.BASE_WIDTH = 0.6
        #388 + 6.5
        self.BASE_WIDTH = 0.44
        #self.TICKS_PER_METER_LATERAL = 103072.0 #Ticks per meter for the slow motors (front and rear)
        #self.TICKS_PER_METER_FRONTAL = 103072.0 #Ticks per meter for the fast motors (left and right)
        self.TICKS_PER_METER_LATERAL = 139071.0 #Ticks per meter for the slow motors (front and rear)
        self.TICKS_PER_METER_FRONTAL = 139071.0 #Ticks per meter for the fast motors (left and right)
        
        #baud_rate_frontal = 38400
        #baud_rate_lateral = 38400
        baud_rate_frontal = 115200
        baud_rate_lateral = 115200

        self.speed_front_left = 0
        self.speed_front_right  = 0
        self.speed_rear  = 0

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
            self.rc_lateral.SpeedM1(self.rc_address_lateral, 0)
            self.rc_lateral.ResetEncoders(self.rc_address_lateral)
            #ROBOCLAW CONFIGURATION CONSTANT
            pos_PID_front_left  = self.rc_frontal.ReadM1PositionPID(self.rc_address_frontal)
            pos_PID_front_right = self.rc_frontal.ReadM2PositionPID(self.rc_address_frontal)
            pos_PID_rear  = self.rc_lateral.ReadM1PositionPID(self.rc_address_lateral)
            vel_PID_front_left  = self.rc_frontal.ReadM1VelocityPID(self.rc_address_frontal)
            vel_PID_front_right = self.rc_frontal.ReadM2VelocityPID(self.rc_address_frontal)
            vel_PID_rear  = self.rc_lateral.ReadM1VelocityPID(self.rc_address_lateral)
            print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos"
            print "MobileBase.->Front Left Motor:  " + str(pos_PID_front_left )
            print "MobileBase.->Front Right Motor: " + str(pos_PID_front_right)
            print "MobileBase.->Rear Motor:  " + str(pos_PID_rear )
            print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS" #QPPS = speed in ticks/s when motor is at full speed
            print "MobileBase.->Front Left Motor:  " + str(vel_PID_front_left )
            print "MobileBase.->Front Right Motor: " + str(vel_PID_front_right)
            print "MobileBase.->Rear Motor:  " + str(vel_PID_rear )
            self.QPPS_FRONT_LEFT  = vel_PID_front_left[4]
            self.QPPS_FRONT_RIGHT = vel_PID_front_right[4]
            self.QPPS_REAR  = vel_PID_rear[4]
            if self.QPPS_FRONT_LEFT != self.QPPS_FRONT_RIGHT or self.QPPS_FRONT_RIGHT != self.QPPS_REAR:
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
            self.QPPS_FRONT_LEFT  = self.TICKS_PER_METER_FRONTAL
            self.QPPS_FRONT_RIGHT = self.TICKS_PER_METER_FRONTAL
            self.QPPS_REAR  = self.TICKS_PER_METER_LATERAL
        
        self.encodm = EncoderOdom(self.TICKS_PER_METER_FRONTAL, self.TICKS_PER_METER_LATERAL, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()
        rospy.sleep(1)
        
        self.pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
        self.pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
        #subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop, queue_size=1);
        self.subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1);
        self.subSimul   = rospy.Subscriber("/simulated", Bool, self.callback_simulated, queue_size = 1);

    def run(self):
        rate = rospy.Rate(30)
        encoder_front_left  = 0
        encoder_front_right = 0
        encoder_rear = 0
        self.newData = False
        self.no_new_data_counter = 5
        jointStates = JointState()
        jointStates.name = ["wheel_front_left_connect", "wheel_front_right_connect", "wheel_back_connect"]
        jointStates.position = [0 , 0, 0]
        while not rospy.is_shutdown():
            if not self.simul:
                if self.newData:
                    self.newData = False
                    
                    #self.speed_left_f  =  int(self.speed_left_f  * self.QPPS_LEFT_F  * 24.0/35.0)
                    #self.speed_right_r = -int(self.speed_right_r * self.QPPS_RIGHT_R * 24.0/35.0)
                    #self.speed_right_f = -int(self.speed_right_f * self.QPPS_RIGHT_F * 24.0/35.0)                                           
                    #self.speed_left_r  = int(self.speed_left_r  * self.QPPS_LEFT_R  * 24.0/35.0)
                    self.speed_front_left  =  -int(self.speed_front_left  * self.TICKS_PER_METER_FRONTAL * 1.25)
                    self.speed_front_right =  -int(self.speed_front_right * self.TICKS_PER_METER_FRONTAL * 1.25)
                    self.speed_rear  =  -int(self.speed_rear  * self.TICKS_PER_METER_LATERAL * 1.25)
            
                    try:
                        # This is a hack way to keep a poorly tuned PID from making noise at speed 0
                        #if self.speed_left is 0 and self.speed_right is 0:
                        #    self.rc_frontal.ForwardM1(self.rc_address_frontal, 0)
                        #    self.rc_frontal.ForwardM2(self.rc_address_frontal, 0)
                        #else:
                        self.rc_frontal.SpeedM1M2(self.rc_address_frontal, self.speed_front_right, self.speed_front_left)
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
                        self.rc_lateral.SpeedM1(self.rc_address_lateral, self.speed_rear)
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
                            except OSError as e:
                                rospy.logerr("Could not stop")
                                rospy.logdebug(e)
                        if self.no_new_data_counter < -1:
                            self.no_new_data_counter = -1;
            
                try:
                    status_front_left, encoder_front_left, crc_front_left = self.rc_frontal.ReadEncM2(self.rc_address_frontal)
                    encoder_front_left = -encoder_front_left
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 frontal OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_front_right, encoder_front_right, crc_front_right = self.rc_frontal.ReadEncM1(self.rc_address_frontal)
                    encoder_front_right = -encoder_front_right
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM2 frontal OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_rear, encoder_rear, crc_rear = self.rc_lateral.ReadEncM1(self.rc_address_lateral)
                    encoder_rear = -encoder_rear
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 lateral OSError: %d", e.errno)
                    rospy.logdebug(e)
            else:
                if not self.newData:
                    self.no_new_data_counter -= 1;
                    if self.no_new_data_counter == 0:
                        self.speed_front_left = 0
                        self.speed_front_right = 0
                        self.speed_rear = 0
                    if self.no_new_data_counter < -1:
                        self.no_new_data_counter = -1;
                else:
                    self.newData = False
                encoder_front_left  = encoder_front_left  + self.speed_front_left   * 0.05 * self.QPPS_FRONT_LEFT
                encoder_front_right = encoder_front_right + self.speed_front_right  * 0.05 * self.QPPS_FRONT_RIGHT
                encoder_rear = encoder_rear + self.speed_rear  * 0.05 * self.QPPS_REAR

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
            self.encodm.update_publish(encoder_front_left, encoder_front_right, encoder_rear)
            jointStates.header.stamp = rospy.Time.now()
            jointStates.position[0] = 0
            jointStates.position[1] = 0
            jointStates.position[2] = 0
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
            except OSError:
                rospy.logerr("Shutdown did not work the frontal motors trying again")
                try:
                    self.rc_lateral.ForwardM1(self.rc_lateral_frontal, 0)
                except OSError as e:
                    rospy.logerr("Could not shutdown the lateral motors!!!!")
                    rospy.logdebug(e)

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = rospy.get_rostime()
        linear_x = -0.866025404 * twist.linear.x + 0.5 * twist.linear.y;
        linear_y = 0.866025404 * twist.linear.x + 0.5 * twist.linear.y;
        angular_z = twist.angular.z
        self.speed_front_left   = linear_x + angular_z * self.BASE_WIDTH/2.0
        self.speed_front_right  = linear_y + angular_z * self.BASE_WIDTH/2.0
        self.speed_rear         = -twist.linear.y + angular_z * self.BASE_WIDTH/2.0

        (self.speed_front_left, self.speed_front_right, self.speed_rear) = self.check_speed_ranges(self.speed_front_left, self.speed_front_right, self.speed_rear)
        self.newData = True
        self.no_new_data_counter = 5;
        #if not self.simul:

    def check_speed_ranges(self, s_front_left, s_front_right, s_rear): #speeds: left, right, front and rear
        max_value = max(abs(s_front_left), abs(s_front_right), abs(s_rear));

        #TODO Check the max value to the frontal motors and pass the parametters how ros params
        if max_value > 2.0:
            s_front_left  = s_front_left  / max_value * 2.0;
            s_front_right = s_front_right / max_value * 2.0;
            s_rear = s_rear / max_value * 2.0;
        return (s_front_left, s_front_right, s_rear);

    def callback_simulated(self, msg):
        self.simul = msg.data;

if __name__ == "__main__":
    try:
        mobileOmniBaseNode = MobileOmniBaseNode()
        mobileOmniBaseNode.run()
    except rospy.ROSInterruptException:
        pass
        rospy.loginfo("Exiting")
