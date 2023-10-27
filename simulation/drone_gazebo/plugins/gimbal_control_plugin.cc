#include "gimbal_control_plugin.hh"

namespace gazebo
{
  // Called by our subscriber when we receive a Quaternion.
  void GimbalControlPlugin::SetCameraGoalAngle(const storm32_gimbal::GimbalOrientationConstPtr &_msg)
  {
    // XXX: Do something with the unlimited field.

    // Change the geometry_msgs::QuaternionStamped into a tf::Quaternion and
    // use built-in tf methods to get the pitch and roll.
    tf::Quaternion q(
        _msg->orientation.x,
        _msg->orientation.y,
        _msg->orientation.z,
        _msg->orientation.w);
    tf::Matrix3x3 m(q);

    // Sets the goal_pitch and goal_roll.
    double yaw_;
    m.getRPY(this->camera_roll, this->camera_pitch, yaw_);

    ROS_DEBUG("Camera (roll, pitch) goal set to (%f, %f)", this->camera_roll, this->camera_pitch);
  }

  void GimbalControlPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    // Kinematic teleportation of each joint to get the desired camera
    // orientation.
    this->roll_joint->SetPosition(0, this->camera_roll);
    this->pitch_joint->SetPosition(0, this->camera_pitch);
  }

  void GimbalControlPlugin::QueueThread()
  {
    // Manually check all the ROS callbacks in an independent thread.
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));

      // Publish current gimbal orientation.
      geometry_msgs::QuaternionStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "gimbal_ref";
      msg.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(
            this->camera_roll, this->camera_pitch, 0);
      this->gimbal_publisher.publish(msg);
    }
  }

  void GimbalControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _model;

    // Extract pointers to the roll and pitch joints from the model.
    this->roll_joint = _model->GetJoint("camera_gimbal_roll");
    this->pitch_joint = _model->GetJoint("camera_gimbal_pitch");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GimbalControlPlugin::OnUpdate, this, _1));

    // Initialize ROS, if it has not already been initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_gooney_client", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_gimbal_controller"));
    this->gimbal_publisher =
      this->rosNode->advertise<geometry_msgs::QuaternionStamped>(
          "/gimbal/controller/camera_orientation", 1);

    ros::SubscribeOptions gimbal_sub_options =
      ros::SubscribeOptions::create<storm32_gimbal::GimbalOrientation>(
          "/gimbal/controller/target_orientation",
          1,
          boost::bind(&GimbalControlPlugin::SetCameraGoalAngle, this, _1),
          ros::VoidPtr(), &this->rosQueue);

    this->gimbal_subscriber = this->rosNode->subscribe(gimbal_sub_options);

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&GimbalControlPlugin::QueueThread, this));
  }

  // Register this plugin with the simulator.
  GZ_REGISTER_MODEL_PLUGIN(GimbalControlPlugin)
}
