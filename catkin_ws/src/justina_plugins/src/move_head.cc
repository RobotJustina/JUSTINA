#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"


namespace gazebo
{
  class MoveHead : public ModelPlugin
  {
    
/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;
//
///// \brief A ROS subscriber
private: ros::Subscriber rosSub;
private: ros::Subscriber subHeadPose;
//
///// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;
//
//// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;

private: bool fmov;
private: float linearX;
private: float linearY;
private: float angularZ;
private: float angularPitch;
float pan_actual;
float pan_objetivo;
float pan_dif;
float tilt_actual;
float tilt_objetivo;
float tilt_dif;
float compX;
float compY;
math::Pose pose;
physics::LinkPtr link;
physics::LinkPtr head_link1;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      link = this->model->GetLink();
      head_link1 = this->model->GetChildLink("head_link2");

      if (!ros::isInitialized())
      {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "move_head_gazebo",
                    ros::init_options::NoSigintHandler);
      }

    this->rosNode.reset(new ros::NodeHandle("move_head_gazebo"));

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->fmov = false;
      this->angularZ=0.1;
      pan_actual = 0.0;
      pan_objetivo = 0.0;
      pan_dif = 0.0;
      tilt_actual = 0.0;
      tilt_objetivo = 0.0;
      tilt_dif = 0.0;
    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
         "/" + this->model->GetName() + "/gazebo_info", 1,
         boost::bind(&MoveHead::OnRosMsg, this, _1),
    ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    
    ros::SubscribeOptions soHeadPose = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
         "/hardware/head/goal_pose", 1,
         boost::bind(&MoveHead::HeadPoseMsg, this, _1),
    ros::VoidPtr(), &this->rosQueue);
    this->subHeadPose = this->rosNode->subscribe(soHeadPose);
    // Spin up the queue helper thread.
    this->rosQueueThread =
    std::thread(std::bind(&MoveHead::QueueThread, this));
     
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MoveHead::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      pose = head_link1->GetWorldPose();
      pan_dif = pan_objetivo - pose.rot.GetAsEuler()[2];
      pan_dif = pan_dif * pan_dif;
      tilt_dif = tilt_objetivo - pose.rot.GetAsEuler()[1];
      tilt_dif = tilt_dif * tilt_dif;
      if(tilt_dif < 0.001)
          this->angularPitch=0.0;
      if(pan_dif < 0.001)
          this->angularZ=0.0;
        head_link1->SetAngularVel(ignition::math::Vector3d(0,this->angularPitch,this->angularZ));

    }
    
    public: void OnRosMsg(const std_msgs::BoolConstPtr &_msg)
    {
      this->fmov=_msg->data;
      std::cout << "link: " << link << std::endl;
      std::cout << "head_link: " << head_link1 << std::endl;
      if(0.9 - pan_actual >= 0)
          this->angularZ = 0.1;
      else
          this->angularZ = -0.1;
      pan_objetivo = 0.9*0.872665;
      pose = head_link1->GetWorldPose();
      std::cout << "pan_objetivo: " << pan_objetivo << std::endl;
    }
    
    public: void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &_msg)
    {
        
        pose = link->GetWorldPose();
        compX = cos(pose.rot.GetAsEuler()[2])*1.6;
        compY = sin(pose.rot.GetAsEuler()[2])*1.6;
        this->linearX=_msg->linear.x * compX;
        this->linearY=_msg->linear.x * compY;
        this->angularZ=_msg->angular.z;
    //std::cout << "rot: " << compX << std::endl;
    }

    public: void HeadPoseMsg(const std_msgs::Float32MultiArray::ConstPtr &_msg)
    {
        //std::cout << "pan: "<< _msg->data[0] << std::endl; 
        //std::cout << "til: "<< _msg->data[1] << std::endl; 
        float tilt_temp;
      if(_msg->data[0] - pan_actual > 0)
          this->angularZ = 2.5;
      else if(_msg->data[0] - pan_actual < 0)
          this->angularZ = -2.5;

      tilt_temp = -1*(_msg->data[1]);
      if(tilt_temp - tilt_actual > 0)
          this->angularPitch = 2.5;
      else if(tilt_temp - tilt_actual < 0)
          this->angularPitch = -2.5;
      tilt_actual = tilt_temp;
      tilt_objetivo = tilt_temp;
      pan_actual = _msg->data[0];
      pan_objetivo = _msg->data[0];
    }

    //  /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
    static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveHead)
}
