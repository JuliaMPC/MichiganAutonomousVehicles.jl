#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

/////////////////////
// to connect to ROS
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// to connect to ROS
////////////////////

// Default
double x = 0;
double y = 0;
double z = 0;

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
          std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
          return;
        }

        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoints()[0];

        // Default to zero velocity
        double velocity = 0;
        //double x,y,z;
        //gazebo::math::Pose pose = this->model->gazebo::physics::ModelState::GetPose();
        //math::Vector3 v(0, 0, 0);

        // Check that the velocity element exists, then read the value
        if (_sdf->HasElement("velocity"))
          velocity = _sdf->Get<double>("velocity");

        this->SetVelocity(velocity);

       // Create the node
       this->node = transport::NodePtr(new transport::Node());
       this->node->Init(this->model->GetWorld()->GetName());

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        // Subscribe to the topic, and register a callback
        this->sub = this->node->Subscribe(topicName,
           &VelodynePlugin::OnMsg, this);

       /////////////////////
       // to connect to ROS
       // Initialize ros, if it has not already been initialized.
       if (!ros::isInitialized())
       {
         int argc = 0;
         char **argv = NULL;
         ros::init(argc, argv, "gazebo_client",
             ros::init_options::NoSigintHandler);
       }

       // Create our ROS node. This acts in a similar manner to
       // the Gazebo node
       this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

       // Create a named topic, and subscribe to it.
       ros::SubscribeOptions so =
         ros::SubscribeOptions::create<std_msgs::Float32>(
             "/" + this->model->GetName() + "/vel_cmd",
             1,
             boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
             ros::VoidPtr(), &this->rosQueue);
       this->rosSub = this->rosNode->subscribe(so);

       // Spin up the queue helper thread.
       this->rosQueueThread =
         std::thread(std::bind(&VelodynePlugin::QueueThread, this));
       // to connect to ROS
       ////////////////////
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
      //this->model->SetPosition(msg.pos);
    }
    /////////////////////
    // to connect to ROS

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetVelocity(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    // to connect to ROS
    ////////////////////

    /// \brief A node used for transport -> setup the messaging passing infrastructure.
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic -> setup the messaging passing infrastructure.
    private: transport::SubscriberPtr sub;

    // Pointer to the model
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
