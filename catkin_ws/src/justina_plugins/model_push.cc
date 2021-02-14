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
#include "geometry_msgs/Twist.h"



namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    
/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;
//
///// \brief A ROS subscriber
private: ros::Subscriber rosSub;
private: ros::Subscriber subCmdVel;
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
float compX;
float compY;
math::Pose pose;
physics::LinkPtr link;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      link = this->model->GetLink();

      if (!ros::isInitialized())
      {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                    ros::init_options::NoSigintHandler);
      }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
         "/" + this->model->GetName() + "/gazebo_test", 1,
         boost::bind(&ModelPush::OnRosMsg, this, _1),
    ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    // Create a topic for cmd vel
    ros::SubscribeOptions socmdvel = ros::SubscribeOptions::create<geometry_msgs::Twist>(
         "/hardware/mobile_base/cmd_vel", 1,
         boost::bind(&ModelPush::cmdvelCallback, this, _1),
    ros::VoidPtr(), &this->rosQueue);
    this->subCmdVel = this->rosNode->subscribe(socmdvel);
    // Spin up the queue helper thread.
    this->rosQueueThread =
    std::thread(std::bind(&ModelPush::QueueThread, this));
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->fmov = false;
      this->linearX=0.0;
      this->linearY=0.0;
      this->angularZ=0.0;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      //if(this->fmov)
      this->model->SetLinearVel(ignition::math::Vector3d(this->linearX, this->linearY, 0));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->angularZ*17.31));
      //pose = link->GetWorldPose();
    //std::cout << "rot: " << pose.rot.GetAsEuler() << std::endl;
        //ROS_INFO("rot: " + pose.rot.GetAsEuler() );
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    

    /// \brief Handle an incoming message from ROS
    ///// \param[in] _msg A float value that is used to set the velocity
    ///// of the Velodyne.
    public: void OnRosMsg(const std_msgs::BoolConstPtr &_msg)
    {
      this->fmov=_msg->data;
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
    //  /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
    static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
