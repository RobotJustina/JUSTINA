/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB Comm ROS.
 *
 * RoAH RSBB Comm ROS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB Comm ROS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB Comm ROS.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __COMM_BENCHMARKS_H__
#define __COMM_BENCHMARKS_H__

#include <memory>
#include <map>
#include <set>
#include <stdexcept>

#include <boost/noncopyable.hpp>
#include <boost/algorithm/string.hpp>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros_roah_rsbb.h>

#include <std_srvs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <roah_rsbb_comm_ros/Benchmark.h>
#include <roah_rsbb_comm_ros/BenchmarkState.h>
#include <roah_rsbb_comm_ros/GoalOMF.h>
#include <roah_rsbb_comm_ros/String.h>
#include <roah_rsbb_comm_ros/ResultHOPF.h>
#include <roah_rsbb_comm_ros/Bool.h>
#include <roah_rsbb_comm_ros/Percentage.h>



using namespace std;
using namespace ros;



template <typename T>
class SenderRepeated
{
    multimap <Time, T> sent_;
    set <T> queue_;

  public:
    SenderRepeated()
    {
    }

    void
    add (T const& t)
    {
      queue_.insert (t);
    }

    void
    fill (Time const& timestamp,
          ::google::protobuf::RepeatedPtrField<T>& field)
    {
      for (auto const& i : queue_) {
        sent_.insert (make_pair (timestamp, i));
      }
      queue_.clear();
      for (auto const& i : sent_) {
        field.Add()->assign (i.second);
      }
    }

    void
    remove (Time const& timestamp)
    {
      sent_.erase (sent_.begin(),
                   sent_.lower_bound (timestamp));
    }

    bool
    empty()
    {
      return sent_.empty() && queue_.empty();
    }
};



class BoolSwitch
{
    ServiceServer srv_;
    ServiceServer on_srv_;
    ServiceServer off_srv_;
    unique_ptr<bool>& state_;

    bool
    set (roah_rsbb_comm_ros::Bool::Request& req,
         roah_rsbb_comm_ros::Bool::Response& res)
    {
      *state_ = req.data;
      return true;
    }

    bool
    on (std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
      *state_ = true;
      return true;
    }

    bool
    off (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& res)
    {
      *state_ = false;
      return true;
    }

  public:
    BoolSwitch (NodeHandle& nh,
                string const& name,
                unique_ptr<bool>& state)
      : srv_ (nh.advertiseService ("/roah_rsbb/devices/" + name + "/set", &BoolSwitch::set, this))
      , on_srv_ (nh.advertiseService ("/roah_rsbb/devices/" + name + "/on", &BoolSwitch::on, this))
      , off_srv_ (nh.advertiseService ("/roah_rsbb/devices/" + name + "/off", &BoolSwitch::off, this))
      , state_ (state)
    {
      state.reset (new bool());
    }
};



class PercentageSwitch
{
    ServiceServer srv_;
    ServiceServer max_srv_;
    ServiceServer min_srv_;
    unique_ptr<uint32_t>& state_;

    bool
    set (roah_rsbb_comm_ros::Percentage::Request& req,
         roah_rsbb_comm_ros::Percentage::Response& res)
    {
      if ( (req.data < 0) || (req.data > 100)) {
        return false;
      }

      *state_ = req.data;
      return true;
    }

    bool
    max (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& res)
    {
      *state_ = 100;
      return true;
    }

    bool
    min (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& res)
    {
      *state_ = 0;
      return true;
    }

  public:
    PercentageSwitch (NodeHandle& nh,
                      string const& name,
                      unique_ptr<uint32_t>& state)
      : srv_ (nh.advertiseService ("/roah_rsbb/devices/" + name + "/set", &PercentageSwitch::set, this))
      , max_srv_ (nh.advertiseService ("/roah_rsbb/devices/" + name + "/max", &PercentageSwitch::max, this))
      , min_srv_ (nh.advertiseService ("/roah_rsbb/devices/" + name + "/min", &PercentageSwitch::min, this))
      , state_ (state)
    {
      state.reset (new uint32_t());
    }
};



class BenchmarkBase
  : boost::noncopyable
{
  protected:
    NodeHandle nh_;

    Publisher benchmark_state_pub_;
    roah_rsbb_comm_ros::BenchmarkState::_benchmark_state_type last_benchmark_state_;

    ServiceServer end_prepare_srv_;
    ServiceServer end_execute_srv_;

    roah_rsbb_msgs::RobotState::State state_;

    boost::function<void() > start_burst_;

    SenderRepeated<string> notifications_;
    SenderRepeated<string> final_command_;
    SenderRepeated<string> activation_event_;
    SenderRepeated<string> visitor_;

    unique_ptr<bool> devices_switch_1_;
    unique_ptr<bool> devices_switch_2_;
    unique_ptr<bool> devices_switch_3_;
    unique_ptr<uint32_t> devices_dimmer_;
    unique_ptr<uint32_t> devices_blinds_;
    unique_ptr<bool> tablet_display_map_;

    Subscriber DEP_messages_saved_sub_;
    Subscriber messages_saved_sub_;
    uint32_t messages_saved_;

    bool should_accept_new_prepare_while_executing_;

    void
    new_state (roah_rsbb_msgs::RobotState::State const& new_state)
    {
      if (state_ == new_state) {
        return;
      }

      state_ = new_state;
      start_burst_();

      roah_rsbb_comm_ros::BenchmarkState::Ptr benchmark_state = boost::make_shared<roah_rsbb_comm_ros::BenchmarkState>();
      switch (new_state) {
        case roah_rsbb_msgs::RobotState_State_STOP:
          cout << "\n\nISTO TA A IR PARA: 0\n\n" << flush;
          benchmark_state->benchmark_state = roah_rsbb_comm_ros::BenchmarkState::STOP;
          break;
        case roah_rsbb_msgs::RobotState_State_PREPARING:
        //break;
        case roah_rsbb_msgs::RobotState_State_WAITING_GOAL:
          cout << "\n\nISTO TA A IR PARA: 1\n\n" << flush;
          benchmark_state->benchmark_state = roah_rsbb_comm_ros::BenchmarkState::PREPARE;
          break;
        case roah_rsbb_msgs::RobotState_State_EXECUTING:
        //break;
        case roah_rsbb_msgs::RobotState_State_RESULT_TX:
          cout << "\n\nISTO TA A IR PARA: 2\n\n" << flush;
          benchmark_state->benchmark_state = roah_rsbb_comm_ros::BenchmarkState::EXECUTE;
          break;
      }
      if (last_benchmark_state_ != benchmark_state->benchmark_state) {
        last_benchmark_state_ = benchmark_state->benchmark_state;
        benchmark_state_pub_.publish (benchmark_state);
      }
    }

    void
    DEP_messages_saved_callback (std_msgs::UInt32::ConstPtr const& msg)
    {
      ROS_ERROR_STREAM_THROTTLE (1, "Usage of /devices/messages_saved is DEPRECATED, the name is wrong. Use /roah_rsbb/messages_saved");
      messages_saved_ = msg->data;
    }

    void
    messages_saved_callback (std_msgs::UInt32::ConstPtr const& msg)
    {
      messages_saved_ = msg->data;
    }

    BenchmarkBase (NodeHandle& nh,
                   boost::function<void() > start_burst)
      : nh_ (nh)
      , benchmark_state_pub_ (nh.advertise<roah_rsbb_comm_ros::BenchmarkState> ("/roah_rsbb/benchmark/state", 1, true))
      , last_benchmark_state_ (roah_rsbb_comm_ros::BenchmarkState::STOP)
      , state_ (roah_rsbb_msgs::RobotState_State_STOP)
      , start_burst_ (start_burst)
      , DEP_messages_saved_sub_ (nh_.subscribe ("/devices/messages_saved", 1, &BenchmarkBase::DEP_messages_saved_callback, this))
      , messages_saved_sub_ (nh_.subscribe ("/roah_rsbb/messages_saved", 1, &BenchmarkBase::messages_saved_callback, this))
      , messages_saved_ (0)
      , should_accept_new_prepare_while_executing_ (false)
    {
      roah_rsbb_comm_ros::BenchmarkState::Ptr benchmark_state = boost::make_shared<roah_rsbb_comm_ros::BenchmarkState>();
      benchmark_state->benchmark_state = last_benchmark_state_;
      benchmark_state_pub_.publish (benchmark_state);
    }

    bool
    default_end_prepare_callback (std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res)
    {
      ROS_INFO_STREAM ("Ending prepare stage, waiting for goal");
      new_state (roah_rsbb_msgs::RobotState_State_WAITING_GOAL);
      return true;
    }

    bool
    default_end_execute_callback (std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res)
    {
      ROS_INFO_STREAM ("Ending execution stage");
      new_state (roah_rsbb_msgs::RobotState_State_RESULT_TX);
      return true;
    }

    virtual void
    advertise_end_prepare()
    {
      end_prepare_srv_ = nh_.advertiseService ("/roah_rsbb/end_prepare", &BenchmarkBase::default_end_prepare_callback, this);
    }

    virtual void
    advertise_end_execute()
    {
      end_execute_srv_ = nh_.advertiseService ("/roah_rsbb/end_execute", &BenchmarkBase::default_end_execute_callback, this);
    }

    virtual void
    receive_goal (roah_rsbb_msgs::BenchmarkState const& msg)
    {
    }

    virtual void
    fill_result (roah_rsbb_msgs::RobotState& msg)
    {
    }

  public:
    typedef std::shared_ptr<BenchmarkBase> Ptr;

    static BenchmarkBase*
    create (uint8_t benchmark,
            NodeHandle& nh,
            boost::function<void() > start_burst);

    static uint8_t
    benchmark_from_string (string const& benchmark);

    static BenchmarkBase*
    create (string const& benchmark,
            NodeHandle& nh,
            boost::function<void() > start_burst)
    {
      return create (benchmark_from_string (benchmark), nh, start_burst);
    }

    virtual ~BenchmarkBase () {}

    void
    receive (roah_rsbb_msgs::BenchmarkState const& msg)
    {
      if (msg.has_acknowledgement()) {
        Time acknowledgement = roah_rsbb::proto_to_ros_time (msg.acknowledgement());
        notifications_.remove (acknowledgement);
        final_command_.remove (acknowledgement);
        activation_event_.remove (acknowledgement);
        visitor_.remove (acknowledgement);
      }

      if ( (state_ != roah_rsbb_msgs::RobotState_State_PREPARING)
           && end_prepare_srv_) {
        end_prepare_srv_ = ServiceServer();
      }
      if ( (state_ != roah_rsbb_msgs::RobotState_State_EXECUTING)
           && end_execute_srv_) {
        end_execute_srv_ = ServiceServer();
      }

      /* cout << "\nLOCAL STATE: " << state_ << endl; */
      /* cout << "RECEIVED STATE: " << msg.benchmark_state() << endl; */
      // State Machine Implementation for externally triggered transitions
      switch (state_) {
        case roah_rsbb_msgs::RobotState_State_STOP:
          switch (msg.benchmark_state()) {
            case roah_rsbb_msgs::BenchmarkState_State_STOP:
              // Keep
              break;
            case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
              ROS_INFO_STREAM ("Prepare for benchmark");
              new_state (roah_rsbb_msgs::RobotState_State_PREPARING);
              advertise_end_prepare();
              break;
            case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
              ROS_ERROR_STREAM ("Received unexpected state from RSBB: Jumped directly from STOP to GOAL_TX");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
              ROS_ERROR_STREAM ("Received unexpected state from RSBB: Jumped directly from STOP to WAITING_RESULT");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            default:
              throw runtime_error ("Received unknown benchmark state from RSBB");
          }
          break;
        case roah_rsbb_msgs::RobotState_State_PREPARING:
          switch (msg.benchmark_state()) {
            case roah_rsbb_msgs::BenchmarkState_State_STOP:
              ROS_WARN_STREAM ("Received unexpected STOP: Halting");
              ROS_INFO_STREAM ("INFO: Received unexpected STOP: Halting");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
              // Keep
              // Will move on when the state is internally changed to WAITING_GOAL
              break;
            case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
              ROS_ERROR_STREAM ("Received unexpected state from RSBB: Jumped directly from PREPARE to GOAL_TX");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
              ROS_ERROR_STREAM ("Received unexpected state from RSBB: Jumped directly from PREPARE to WAITING_RESULT");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            default:
              throw runtime_error ("Received unknown benchmark state from RSBB");
          }
          break;
        case roah_rsbb_msgs::RobotState_State_WAITING_GOAL:
          switch (msg.benchmark_state()) {
            case roah_rsbb_msgs::BenchmarkState_State_STOP:
              ROS_WARN_STREAM ("Received unexpected STOP: Halting");
              ROS_INFO_STREAM ("INFO: Received unexpected STOP: Halting");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
              // Keep
              break;
            case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
              ROS_INFO_STREAM ("Receiving goal and starting benchmark");
              receive_goal (msg);
              new_state (roah_rsbb_msgs::RobotState_State_EXECUTING);
              advertise_end_execute();
              break;
            case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
              ROS_INFO_STREAM ("No goal, starting benchmark");
              new_state (roah_rsbb_msgs::RobotState_State_EXECUTING);
              advertise_end_execute();
              break;
            default:
              throw runtime_error ("Received unknown benchmark state from RSBB");
          }
          break;
        case roah_rsbb_msgs::RobotState_State_EXECUTING:
          switch (msg.benchmark_state()) {
            case roah_rsbb_msgs::BenchmarkState_State_STOP:
              ROS_WARN_STREAM ("Received unexpected STOP: Halting");
              ROS_INFO_STREAM ("INFO: Received unexpected STOP: Halting");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
              // Keep
              // RSBB still hasn't received my EXECUTING
              if (should_accept_new_prepare_while_executing_) {
                cout << "\n\nA TIMEOUT HAS OCCURRED\n\n";
                end_execute_srv_ = ServiceServer();
                new_state (roah_rsbb_msgs::RobotState_State_PREPARING);
                advertise_end_prepare();
              }
              break;
            case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
              // Keep
              // Will move on when the state is internally changed to RESULT_TX
              break;
            case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
              // Keep
              // Will move on when the state is internally changed to RESULT_TX
              break;
            default:
              throw runtime_error ("Received unknown benchmark state from RSBB");
          }
          break;
        case roah_rsbb_msgs::RobotState_State_RESULT_TX:
          switch (msg.benchmark_state()) {
            case roah_rsbb_msgs::BenchmarkState_State_STOP:
              ROS_INFO_STREAM ("End of benchmark");
              new_state (roah_rsbb_msgs::RobotState_State_STOP);
              break;
            case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
              ROS_INFO_STREAM ("New goal, prepare");
              new_state (roah_rsbb_msgs::RobotState_State_PREPARING);
              advertise_end_prepare();
              break;
            case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
              // Keep
              break;
            case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
              // Keep
              break;
            default:
              throw runtime_error ("Received unknown benchmark state from RSBB");
          }
          break;
        default:
          throw logic_error ("Invalid internal benchmark state");
      }
    }

    void
    fill (roah_rsbb_msgs::RobotState& msg)
    {
      Time timestamp = roah_rsbb::proto_to_ros_time (msg.time());

      msg.set_messages_saved (messages_saved_);
      msg.set_robot_state (state_);
      notifications_.fill (timestamp, *msg.mutable_notifications());
      final_command_.fill (timestamp, *msg.mutable_final_command());
      activation_event_.fill (timestamp, *msg.mutable_activation_event());
      visitor_.fill (timestamp, *msg.mutable_visitor());
      if (devices_switch_1_) {
        msg.set_devices_switch_1 (*devices_switch_1_);
      }
      if (devices_switch_2_) {
        msg.set_devices_switch_2 (*devices_switch_2_);
      }
      if (devices_switch_3_) {
        msg.set_devices_switch_3 (*devices_switch_3_);
      }
      if (devices_dimmer_) {
        msg.set_devices_dimmer (*devices_dimmer_);
      }
      if (devices_blinds_) {
        msg.set_devices_blinds (*devices_blinds_);
      }
      if (tablet_display_map_) {
        msg.set_tablet_display_map (*tablet_display_map_);
      }

      if (state_ == roah_rsbb_msgs::RobotState_State_RESULT_TX) {
        fill_result (msg);
      }
    }
};



class BenchmarkHGTKMH
  : public BenchmarkBase
{
  public:
    BenchmarkHGTKMH (NodeHandle& nh,
                     boost::function<void() > start_burst)
      : BenchmarkBase (nh, start_burst)
    {
    }
};



class BenchmarkHWV
  : public BenchmarkBase
{
    ServiceServer notifications_srv_;
    ServiceServer activation_event_srv_;
    ServiceServer visitor_srv_;

    bool
    notifications_callback (roah_rsbb_comm_ros::String::Request& req,
                            roah_rsbb_comm_ros::String::Response& res)
    {
      notifications_.add (req.data);
      return true;
    }

    bool
    activation_event_callback (roah_rsbb_comm_ros::String::Request& req,
                               roah_rsbb_comm_ros::String::Response& res)
    {
      activation_event_.add (req.data);
      return true;
    }

    bool
    visitor_callback (roah_rsbb_comm_ros::String::Request& req,
                      roah_rsbb_comm_ros::String::Response& res)
    {
      visitor_.add (req.data);
      return true;
    }

  public:
    BenchmarkHWV (NodeHandle& nh,
                  boost::function<void() > start_burst)
      : BenchmarkBase (nh, start_burst)
      , notifications_srv_ (nh.advertiseService ("/roah_rsbb/notifications", &BenchmarkHWV::notifications_callback, this))
      , activation_event_srv_ (nh.advertiseService ("/roah_rsbb/activation_event", &BenchmarkHWV::activation_event_callback, this))
      , visitor_srv_ (nh.advertiseService ("/roah_rsbb/visitor", &BenchmarkHWV::visitor_callback, this))
    {
    }
};



class BenchmarkHCFGAC
  : public BenchmarkBase
{
    ServiceServer final_command_srv_;
    BoolSwitch switch_1_;
    BoolSwitch switch_2_;
    BoolSwitch switch_3_;
    PercentageSwitch dimmer_;
    PercentageSwitch blinds_;
    ServiceServer tablet_buttons_srv_;
    ServiceServer tablet_map_srv_;

    bool
    final_command_callback (roah_rsbb_comm_ros::String::Request& req,
                            roah_rsbb_comm_ros::String::Response& res)
    {
      final_command_.add (req.data);
      return true;
    }

    bool
    tablet_buttons (std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& res)
    {
      *tablet_display_map_ = false;
      return true;
    }

    bool
    tablet_map (std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& res)
    {
      *tablet_display_map_ = true;
      return true;
    }

  public:
    BenchmarkHCFGAC (NodeHandle& nh, boost::function<void() > start_burst)
      : BenchmarkBase (nh, start_burst)
      , final_command_srv_ (nh.advertiseService ("/roah_rsbb/final_command", &BenchmarkHCFGAC::final_command_callback, this))
      , switch_1_ (nh, "switch_1", devices_switch_1_)
      , switch_2_ (nh, "switch_2", devices_switch_2_)
      , switch_3_ (nh, "switch_3", devices_switch_3_)
      , dimmer_ (nh, "dimmer", devices_dimmer_)
      , blinds_ (nh, "blinds", devices_blinds_)
      , tablet_buttons_srv_ (nh.advertiseService ("/roah_rsbb/tablet/buttons", &BenchmarkHCFGAC::tablet_buttons, this))
      , tablet_map_srv_ (nh.advertiseService ("/roah_rsbb/tablet/map", &BenchmarkHCFGAC::tablet_map, this))
    {
      tablet_display_map_.reset (new bool());
    }
};



class BenchmarkHOPF
  : public BenchmarkBase
{
    roah_rsbb_comm_ros::ResultHOPF::Request result_;

    bool
    end_execute_callback (roah_rsbb_comm_ros::ResultHOPF::Request& req,
                          roah_rsbb_comm_ros::ResultHOPF::Response& res)
    {
      ROS_INFO_STREAM ("Ending execution stage");
      result_ = req;
      new_state (roah_rsbb_msgs::RobotState_State_RESULT_TX);
      return true;
    }

  protected:
    virtual void
    advertise_end_execute()
    {
      end_execute_srv_ = nh_.advertiseService ("/roah_rsbb/end_execute", &BenchmarkHOPF::end_execute_callback, this);
    }

  public:
    BenchmarkHOPF (NodeHandle& nh,
                   boost::function<void() > start_burst)
      : BenchmarkBase (nh, start_burst)
    {
    }

    virtual void
    fill_result (roah_rsbb_msgs::RobotState& msg)
    {
      msg.set_object_class (result_.object_class);
      msg.set_object_name (result_.object_name);
      msg.set_object_pose_x (result_.object_pose.x);
      msg.set_object_pose_y (result_.object_pose.y);
      msg.set_object_pose_theta (result_.object_pose.theta);
      
      YAML::Node result_node;
      
      result_node["class"] = result_.object_class;
      result_node["instance"] = result_.object_name;
      result_node["x"] = result_.object_pose.x;
      result_node["y"] = result_.object_pose.y;
      result_node["theta"] = result_.object_pose.theta;
      
      YAML::Emitter emitted_stream;
      
      emitted_stream << result_node;
      
      msg.set_generic_result(emitted_stream.c_str());
      
    }
};



class BenchmarkHNF
  : public BenchmarkBase
{
    ServiceServer notifications_srv_;
    Publisher goal_pub_;
    // roah_rsbb_comm_ros::GoalOMF::Ptr goal_msg_;
    geometry_msgs::Pose2D::Ptr goal_msg_;

    bool
    notifications_callback (roah_rsbb_comm_ros::String::Request& req,
                            roah_rsbb_comm_ros::String::Response& res)
    {
      notifications_.add (req.data);
      return true;
    }

  public:
    BenchmarkHNF (NodeHandle& nh,
                  boost::function<void() > start_burst)
      : BenchmarkBase (nh, start_burst)
      , notifications_srv_ (nh.advertiseService ("/roah_rsbb/notifications", &BenchmarkHNF::notifications_callback, this))
      //, goal_pub_ (nh.advertise<roah_rsbb_comm_ros::GoalOMF> ("/roah_rsbb/goal", 1, true))
      , goal_pub_ (nh.advertise<geometry_msgs::Pose2D> ("/roah_rsbb/goal", 1, true))
    {
      should_accept_new_prepare_while_executing_ = true;
    }

    virtual void
    receive_goal (roah_rsbb_msgs::BenchmarkState const& proto_msg)
    {
      goal_msg_ = boost::make_shared<geometry_msgs::Pose2D>();

      YAML::Node goal_payload = YAML::Load(proto_msg.generic_goal());

      goal_msg_->x = goal_payload[0].as<double>();
      goal_msg_->y = goal_payload[1].as<double>();
      goal_msg_->theta = goal_payload[2].as<double>();

      cout << "RECEIVING GOAL!!! " << endl;
      cout << "\tx: " << goal_msg_->x << endl;
      cout << "\ty: " << goal_msg_->y << endl;
      cout << "\ttheta: " << goal_msg_->theta << endl;

      goal_pub_.publish (goal_msg_);

    }
};


class BenchmarkSTB
  : public BenchmarkBase
{

    Publisher goal_pub_;

//    bool
//    end_execute_callback (roah_rsbb_comm_ros::ResultHOPF::Request& req,
//                          roah_rsbb_comm_ros::ResultHOPF::Response& res)
//    {
//      ROS_INFO_STREAM ("Ending execution stage");
//      result_ = req;
//      new_state (roah_rsbb_msgs::RobotState_State_RESULT_TX);
//      return true;
//    }

  protected:
//    virtual void
//    advertise_end_execute()
//    {
//      end_execute_srv_ = nh_.advertiseService ("/roah_rsbb/end_execute", &BenchmarkHOPF::end_execute_callback, this);
//    }

public:
	BenchmarkSTB(NodeHandle& nh, boost::function<void()> start_burst) :
			BenchmarkBase(nh, start_burst),
			goal_pub_(nh.advertise<std_msgs::String>("/roah_rsbb/goal", 1, true)) {
	}

    virtual void
    receive_goal (roah_rsbb_msgs::BenchmarkState const& proto_msg) {

    	std_msgs::String goalMsg = std_msgs::String();
    	goalMsg.data = proto_msg.generic_goal();

    	cout << "RECEIVING GOAL!!!" << endl;
    	cout << goalMsg.data << endl;

    	goal_pub_.publish (goalMsg);

    }

    virtual void
    fill_result (roah_rsbb_msgs::RobotState& msg)
    {
      msg.set_generic_result("SOME RESULT");
    }
};




class BenchmarkHSUF
  : public BenchmarkBase
{
  public:
    BenchmarkHSUF (NodeHandle& nh,
                   boost::function<void() > start_burst)
      : BenchmarkBase (nh, start_burst)
    {
    }
};



BenchmarkBase*
BenchmarkBase::create (uint8_t benchmark,
                       NodeHandle& nh,
                       boost::function<void() > start_burst)
{
  switch (benchmark) {
    case roah_rsbb_comm_ros::Benchmark::NONE:
      return nullptr;
    case roah_rsbb_comm_ros::Benchmark::HGTKMH:
      return new BenchmarkHGTKMH (nh, start_burst);
    case roah_rsbb_comm_ros::Benchmark::HWV:
      return new BenchmarkHWV (nh, start_burst);
    case roah_rsbb_comm_ros::Benchmark::HCFGAC:
      return new BenchmarkHCFGAC (nh, start_burst);
    case roah_rsbb_comm_ros::Benchmark::HOPF:
      return new BenchmarkHOPF (nh, start_burst);
    case roah_rsbb_comm_ros::Benchmark::HNF:
      return new BenchmarkHNF (nh, start_burst);
    case roah_rsbb_comm_ros::Benchmark::STB:
      return new BenchmarkSTB (nh, start_burst);
    case roah_rsbb_comm_ros::Benchmark::HSUF:
      return new BenchmarkHSUF (nh, start_burst);
    default:
      ROS_ERROR_STREAM ("Cannot initialize benchmark of type " << static_cast<int> (benchmark));
      return nullptr;
  }
}



uint8_t
BenchmarkBase::benchmark_from_string (string const& benchmark)
{
  string upper = boost::to_upper_copy (benchmark);

  if (upper == "NONE") {
    return roah_rsbb_comm_ros::Benchmark::NONE;
  }
  else if (upper == "HGTKMH") {
    return roah_rsbb_comm_ros::Benchmark::HGTKMH;
  }
  else if (upper == "HWV") {
    return roah_rsbb_comm_ros::Benchmark::HWV;
  }
  else if (upper == "HCFGAC") {
    return roah_rsbb_comm_ros::Benchmark::HCFGAC;
  }
  else if (upper == "HOPF") {
    return roah_rsbb_comm_ros::Benchmark::HOPF;
  }
  else if (upper == "HNF") {
    return roah_rsbb_comm_ros::Benchmark::HNF;
  }
  else if (upper == "STB") {
    return roah_rsbb_comm_ros::Benchmark::STB;
  }
  else if (upper == "HSUF") {
    return roah_rsbb_comm_ros::Benchmark::HSUF;
  }

  ROS_ERROR_STREAM ("Unrecognized benchmark of type \"" << benchmark << "\"");
  return roah_rsbb_comm_ros::Benchmark::NONE;
}

#endif
