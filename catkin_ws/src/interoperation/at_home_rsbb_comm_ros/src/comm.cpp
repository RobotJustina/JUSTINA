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

#include <sstream>
#include <memory>
#include <mutex>
#include <atomic>

#include <boost/noncopyable.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <ros_roah_rsbb.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <roah_rsbb_comm_ros/Benchmark.h>
#include <roah_rsbb_comm_ros/DevicesState.h>
#include <roah_rsbb_comm_ros/TabletState.h>
#include <roah_rsbb_comm_ros/Override.h>

#include "comm_benchmarks.h"



using namespace std;
using namespace ros;



inline bool
same (roah_rsbb_comm_ros::Benchmark const& a,
      roah_rsbb_comm_ros::Benchmark const& b)
{
  return (a.benchmark == b.benchmark);
}



inline bool
same (roah_rsbb_comm_ros::DevicesState const& a,
      roah_rsbb_comm_ros::DevicesState const& b)
{
  return (a.bell == b.bell)
         && (a.switch_1 == b.switch_1)
         && (a.switch_2 == b.switch_2)
         && (a.switch_3 == b.switch_3)
         && (a.dimmer == b.dimmer)
         && (a.blinds == b.blinds)
         && (a.door_win_detect == b.door_win_detect);
}



inline bool
same (roah_rsbb_comm_ros::TabletState const& a,
      roah_rsbb_comm_ros::TabletState const& b)
{
  return (a.display_map == b.display_map)
         && (a.call_time == b.call_time)
         && (a.position_time == b.position_time)
         && (a.position_x == b.position_x)
         && (a.position_y == b.position_y);
}



inline string
to_string (roah_rsbb_msgs::RobotBeacon const& msg)
{
  ostringstream ss;
  ss << "team_name: " << msg.team_name()
     << " ; robot_name: " << msg.robot_name()
     << " ; time: " << msg.time().sec() << "," << msg.time().nsec();
  return ss.str();
}



inline string
to_string (roah_rsbb_msgs::RobotState const& msg)
{
  ostringstream ss;
  ss << "time: " << msg.time().sec() << "," << msg.time().nsec()
     << " ; messages_saved: " << msg.messages_saved()
     << " ; robot_state: ";
  switch (msg.robot_state()) {
    case roah_rsbb_msgs::RobotState_State_STOP:
      ss << "STOP";
      break;
    case roah_rsbb_msgs::RobotState_State_PREPARING:
      ss << "PREPARING";
      break;
    case roah_rsbb_msgs::RobotState_State_WAITING_GOAL:
      ss << "WAITING_GOAL";
      break;
    case roah_rsbb_msgs::RobotState_State_EXECUTING:
      ss << "EXECUTING";
      break;
    case roah_rsbb_msgs::RobotState_State_RESULT_TX:
      ss << "RESULT_TX";
      break;
  }
  ss << " ; notifications: [ ";
  for (auto const& i : msg.notifications()) {
    ss << "\"" << i << "\" ";
  }
  ss << "] ; activation_event: [ ";
  for (auto const& i : msg.activation_event()) {
    ss << "\"" << i << "\" ";
  }
  ss << "] ; visitor: [ ";
  for (auto const& i : msg.visitor()) {
    ss << "\"" << i << "\" ";
  }
  ss << "] ; final_command: [ ";
  for (auto const& i : msg.final_command()) {
    ss << "\"" << i << "\" ";
  }
  ss << "]";
  if (msg.has_devices_switch_1()) {
    ss << " ; devices_switch_1: " << (msg.devices_switch_1() ? "ON" : "OFF");
  }
  if (msg.has_devices_switch_2()) {
    ss << " ; devices_switch_2: " << (msg.devices_switch_2() ? "ON" : "OFF");
  }
  if (msg.has_devices_switch_3()) {
    ss << " ; devices_switch_3: " << (msg.devices_switch_3() ? "ON" : "OFF");
  }
  if (msg.has_devices_dimmer()) {
    ss << " ; devices_dimmer: " << msg.devices_dimmer();
  }
  if (msg.has_devices_blinds()) {
    ss << " ; devices_blinds: " << msg.devices_blinds();
  }
  if (msg.has_tablet_display_map()) {
    ss << " ; tablet_display_map: " << (msg.devices_switch_1() ? "MAP" : "BUTTONS");
  }
  if (msg.has_object_class()) {
    ss << " ; object_class: " << msg.object_class();
  }
  if (msg.has_object_name()) {
    ss << " ; object_name: " << msg.object_name();
  }
  if (msg.has_object_pose_x()) {
    ss << " ; object_pose_x: " << msg.object_pose_x();
  }
  if (msg.has_object_pose_y()) {
    ss << " ; object_pose_y: " << msg.object_pose_y();
  }
  if (msg.has_object_pose_theta()) {
    ss << " ; object_pose_theta: " << msg.object_pose_theta();
  }
  return ss.str();
}



class Comm
  : boost::noncopyable
  , roah_rsbb::RosPublicChannel
{
    NodeHandle nh_;

    Timer beacon_timer_;
    Timer state_timer_;
    unsigned state_burst_;

    Publisher benchmark_pub_;
    roah_rsbb_comm_ros::Benchmark::ConstPtr last_benchmark_;
    Publisher devices_state_pub_;
    Publisher devices_bell_pub_;
    roah_rsbb_comm_ros::DevicesState::ConstPtr last_devices_state_;
    Publisher tablet_state_pub_;
    Publisher tablet_call_pub_;
    Publisher tablet_position_pub_;
    roah_rsbb_comm_ros::TabletState::ConstPtr last_tablet_state_;

    ServiceServer override_srv_;
    bool override_;

    uint8_t current_benchmark_;
    unique_ptr<BenchmarkBase> benchmark_;
    unique_ptr<roah_rsbb::RosPrivateChannel> private_channel_;

    void
    transmit_beacon (const TimerEvent&)
    {
      if (private_channel_ || override_) {
        return;
      }

      roah_rsbb_msgs::RobotBeacon msg;
      msg.set_team_name (param_direct<string> ("~team_name", "DefaultTeam"));
      msg.set_robot_name (param_direct<string> ("~robot_name", "DefaultRobot"));
      roah_rsbb::now (msg.mutable_time());

      ROS_DEBUG_STREAM ("Transmitting beacon: " << to_string (msg));
      send (msg);
    }

    void
    transmit_state (const TimerEvent&)
    {
      if (! (private_channel_ || override_)) {
        return;
      }

      if (state_burst_ > 0) {
        -- state_burst_;
        if (state_burst_ == 0) {
          state_timer_.setPeriod (Duration (1, 0));
        }
      }

      roah_rsbb_msgs::RobotState msg;
      roah_rsbb::now (msg.mutable_time());
      if (benchmark_) {
        benchmark_->fill (msg);
      }
      else {
        msg.set_messages_saved (0);
        msg.set_robot_state (roah_rsbb_msgs::RobotState_State_STOP);
      }

      if (! override_) {
        ROS_DEBUG_STREAM ("Transmitting robot state: " << to_string (msg));
        private_channel_->send (msg);
      }
      else {
        ROS_DEBUG_STREAM ("If not testing, this robot state would be transmitted: " << to_string (msg));
      }
    }

    void
    start_burst()
    {
      getGlobalCallbackQueue()->addCallback (boost::make_shared<roah_rsbb::CallbackItem> (boost::bind (&Comm::transmit_state, this, TimerEvent())));
      state_timer_.stop();
      state_burst_ = 10;
      state_timer_.setPeriod (Duration (0.05));
      state_timer_.start();
    }

    bool
    override_callback (roah_rsbb_comm_ros::Override::Request& req,
                       roah_rsbb_comm_ros::Override::Response& res)
    {
      if (req.benchmark_type == roah_rsbb_comm_ros::Benchmark::NONE) {
        current_benchmark_ = roah_rsbb_comm_ros::Benchmark::NONE;
        benchmark_.reset();
        roah_rsbb_comm_ros::Benchmark::Ptr bs = boost::make_shared<roah_rsbb_comm_ros::Benchmark>();
        bs->benchmark = roah_rsbb_comm_ros::Benchmark::NONE;
        last_benchmark_ = bs;
        benchmark_pub_.publish (last_benchmark_);

        override_ = false;
        return true;
      }

      private_channel_.reset();
      override_ = true;

      if (current_benchmark_ != req.benchmark_type) {
        current_benchmark_ = req.benchmark_type;
        benchmark_.reset();
        benchmark_.reset (BenchmarkBase::create (req.benchmark_type, nh_, boost::bind (&Comm::start_burst, this)));
        roah_rsbb_comm_ros::Benchmark::Ptr bs = boost::make_shared<roah_rsbb_comm_ros::Benchmark>();
        bs->benchmark = req.benchmark_type;
        benchmark_pub_.publish (bs);
        last_benchmark_ = bs;
      }

      roah_rsbb_msgs::BenchmarkState msg;
      msg.set_benchmark_type ("benchmark_type string in proto message should not be needed by specific benchmark implementation");
      switch (req.benchmark_state) {
        case roah_rsbb_comm_ros::Override::Request::STOP:
          msg.set_benchmark_state (roah_rsbb_msgs::BenchmarkState_State_STOP);
          break;
        case roah_rsbb_comm_ros::Override::Request::PREPARE:
          msg.set_benchmark_state (roah_rsbb_msgs::BenchmarkState_State_PREPARE);
          break;
        case roah_rsbb_comm_ros::Override::Request::GOAL_TX:
          msg.set_benchmark_state (roah_rsbb_msgs::BenchmarkState_State_GOAL_TX);
          break;
        case roah_rsbb_comm_ros::Override::Request::WAITING_RESULT:
          msg.set_benchmark_state (roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT);
          break;
      }
      roah_rsbb::ros_to_proto_time (req.acknowledgement, msg.mutable_acknowledgement());
      // for (auto const& i : req.initial_state) {
      //   msg.mutable_initial_state()->Add (i);
      // }
      // for (auto const& i : req.switches) {
      //   msg.mutable_switches()->Add (i);
      // }
      benchmark_->receive (msg);
      return true;
    }

    void
    receive_benchmark_state (boost::asio::ip::udp::endpoint endpoint,
                             uint16_t comp_id,
                             uint16_t msg_type,
                             std::shared_ptr<const roah_rsbb_msgs::BenchmarkState> msg)
    {
      if (override_) {
        return;
      }

      uint8_t benchmark = BenchmarkBase::benchmark_from_string (msg->benchmark_type());

      if (current_benchmark_ != benchmark) {
        current_benchmark_ = benchmark;
        benchmark_.reset();
        benchmark_.reset (BenchmarkBase::create (benchmark, nh_, boost::bind (&Comm::start_burst, this)));
        roah_rsbb_comm_ros::Benchmark::Ptr bs = boost::make_shared<roah_rsbb_comm_ros::Benchmark>();
        bs->benchmark = benchmark;
        benchmark_pub_.publish (bs);
        last_benchmark_ = bs;
      }

      if (benchmark_) {
        benchmark_->receive (*msg);
      }
    }

    void
    receive_robot_state (boost::asio::ip::udp::endpoint endpoint,
                         uint16_t comp_id,
                         uint16_t msg_type,
                         std::shared_ptr<const roah_rsbb_msgs::RobotState> msg)
    {
      ROS_ERROR_STREAM_THROTTLE (1,
                                 "Detected another robot in the private channel: " << endpoint.address().to_string()
                                 << ":" << endpoint.port()
                                 << ", COMP_ID " << comp_id
                                 << ", MSG_TYPE " << msg_type << endl);
    }

    void
    receive_rsbb_beacon (boost::asio::ip::udp::endpoint endpoint,
                         uint16_t comp_id,
                         uint16_t msg_type,
                         std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> rsbb_beacon)
    {
      if (override_) {
        return;
      }

      roah_rsbb_comm_ros::DevicesState::Ptr ds = boost::make_shared<roah_rsbb_comm_ros::DevicesState>();
      ds->bell = roah_rsbb::proto_to_ros_time (rsbb_beacon->devices_bell());
      ds->switch_1 = rsbb_beacon->devices_switch_1();
      ds->switch_2 = rsbb_beacon->devices_switch_2();
      ds->switch_3 = rsbb_beacon->devices_switch_3();
      ds->dimmer = rsbb_beacon->devices_dimmer();
      ds->blinds = rsbb_beacon->devices_blinds();
      ds->door_win_detect = rsbb_beacon->devices_door_win_detect();
      if ( (! last_devices_state_)
           || (! same (*last_devices_state_, *ds))) {
        if (last_devices_state_) {
          if (last_devices_state_->bell != ds->bell) {
            devices_bell_pub_.publish (std_msgs::Empty());
          }
        }
        devices_state_pub_.publish (ds);
        last_devices_state_ = ds;
      }

      roah_rsbb_comm_ros::TabletState::Ptr ts = boost::make_shared<roah_rsbb_comm_ros::TabletState>();
      ts->display_map = rsbb_beacon->tablet_display_map();
      ts->call_time = roah_rsbb::proto_to_ros_time (rsbb_beacon->tablet_call_time());
      ts->position_time = roah_rsbb::proto_to_ros_time (rsbb_beacon->tablet_position_time());
      ts->position_x = rsbb_beacon->tablet_position_x();
      ts->position_y = rsbb_beacon->tablet_position_y();
      if ( (! last_tablet_state_)
           || (! same (*last_tablet_state_, *ts))) {
        if (last_tablet_state_) {
          if (last_tablet_state_->call_time != ts->call_time) {
            tablet_call_pub_.publish (std_msgs::Empty());
          }
          if (last_tablet_state_->position_time != ts->position_time) {
            geometry_msgs::Pose2D::Ptr pos = boost::make_shared<geometry_msgs::Pose2D>();
            pos->x = ts->position_x;
            pos->y = ts->position_y;
            pos->theta = 0;
            tablet_position_pub_.publish (pos);
          }
        }
        tablet_state_pub_.publish (ts);
        last_tablet_state_ = ts;
      }

      string const team_name = param_direct<string> ("~team_name", "DefaultTeam");
      string const robot_name = param_direct<string> ("~robot_name", "Robot");
      for (auto const& bt : rsbb_beacon->benchmarking_teams()) {
        if ( (bt.team_name() == team_name)
             && (bt.robot_name() == robot_name)) {
          string connect_addr = endpoint.address().to_string();
          unsigned short connect_port = bt.rsbb_port();
          if ( (! private_channel_)
               || (private_channel_->host() != connect_addr)
               || (private_channel_->port() != connect_port)) {
            private_channel_.reset();
            private_channel_.reset (new roah_rsbb::RosPrivateChannel (connect_addr,
                                    connect_port,
                                    param_direct<string> ("~rsbb_key", "DefaultKey"),
                                    param_direct<string> ("~rsbb_cypher", "aes-128-cbc")));
            private_channel_->set_benchmark_state_callback (&Comm::receive_benchmark_state, this);
            private_channel_->set_robot_state_callback (&Comm::receive_robot_state, this);
          }
          return;
        }
      }
      current_benchmark_ = roah_rsbb_comm_ros::Benchmark::NONE;
      private_channel_.reset();
      roah_rsbb_comm_ros::Benchmark::Ptr bs = boost::make_shared<roah_rsbb_comm_ros::Benchmark>();
      bs->benchmark = roah_rsbb_comm_ros::Benchmark::NONE;
      if (! same (*last_benchmark_, *bs)) {
        benchmark_pub_.publish (bs);
        last_benchmark_ = bs;
      }
    }

    void
    receive_robot_beacon (boost::asio::ip::udp::endpoint endpoint,
                          uint16_t comp_id,
                          uint16_t msg_type,
                          std::shared_ptr<const roah_rsbb_msgs::RobotBeacon> msg)
    {
      ROS_DEBUG_STREAM ("Received peer RobotBeacon from " << endpoint.address().to_string()
                        << ":" << endpoint.port()
                        << ", COMP_ID " << comp_id
                        << ", MSG_TYPE " << msg_type
                        << ", team_name: " << msg->team_name()
                        << ", robot_name: " << msg->robot_name());
    }

  public:
    Comm()
      : roah_rsbb::RosPublicChannel (param_direct<string> ("~rsbb_host", "10.0.0.1"),
                                     param_direct<int> ("~rsbb_port", 6666))
      , nh_()
      , beacon_timer_ (nh_.createTimer (Duration (1, 0), &Comm::transmit_beacon, this, false, true))
      , state_timer_ (nh_.createTimer (Duration (1, 0), &Comm::transmit_state, this, false, true))
      , state_burst_ (0)
      , benchmark_pub_ (nh_.advertise<roah_rsbb_comm_ros::Benchmark> ("/roah_rsbb/benchmark", 1, true))
      , devices_state_pub_ (nh_.advertise<roah_rsbb_comm_ros::DevicesState> ("/roah_rsbb/devices/state", 1, true))
      , devices_bell_pub_ (nh_.advertise<std_msgs::Empty> ("/roah_rsbb/devices/bell", 1, false))
      , tablet_state_pub_ (nh_.advertise<roah_rsbb_comm_ros::TabletState> ("/roah_rsbb/tablet/state", 1, true))
      , tablet_call_pub_ (nh_.advertise<std_msgs::Empty> ("/roah_rsbb/tablet/call", 1, false))
      , tablet_position_pub_ (nh_.advertise<geometry_msgs::Pose2D> ("/roah_rsbb/tablet/position", 1, false))
      , override_srv_ (nh_.advertiseService ("/roah_rsbb/override", &Comm::override_callback, this))
      , override_ (false)
      , current_benchmark_ (roah_rsbb_comm_ros::Benchmark::NONE)
    {
      auto tmp_benchmark_ = boost::make_shared<roah_rsbb_comm_ros::Benchmark>();
      tmp_benchmark_->benchmark = roah_rsbb_comm_ros::Benchmark::NONE;
      last_benchmark_ = tmp_benchmark_;
      benchmark_pub_.publish (last_benchmark_);

      set_rsbb_beacon_callback (&Comm::receive_rsbb_beacon, this);
      set_robot_beacon_callback (&Comm::receive_robot_beacon, this);
    }

    ~Comm()
    {
      if (private_channel_) {
        private_channel_->signal_benchmark_state_received().disconnect_all_slots();
        private_channel_->signal_robot_state_received().disconnect_all_slots();
      }
      signal_rsbb_beacon_received().disconnect_all_slots();
      signal_robot_beacon_received().disconnect_all_slots();
    }
};



int main (int argc, char** argv)
{
  init (argc, argv, "roah_rsbb_comm_ros");
  NodeHandle nh;

  if (! param::has ("~team_name")) {
    ROS_FATAL_STREAM ("At least the parameter \"team_name\" must be set before running!");
    shutdown();
    abort();
  }
  if (! param::has ("~rsbb_key")) {
    ROS_ERROR_STREAM ("The parameter \"rsbb_key\" should be set before running");
  }

  Comm node;

  spin();
  return 0;
}
