/**
 * gimbal_control_plugin.hh
 *
 * Model plugin to control the camera gimbal pitch and roll for gooney4.
 */

#ifndef __DRONE_GAZEBO_GOONEY_PLUGIN_H_
#define __DRONE_GAZEBO_GOONEY_PLUGIN_H_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <stdio.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "storm32_gimbal/GimbalOrientation.h"

namespace gazebo{
  class GimbalControlPlugin : public ModelPlugin
  {
  public:
    // Called by gazebo when plugin is loaded.
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

    // Called by the world update start event.
    void OnUpdate(const common::UpdateInfo & /*_info*/);

    /// \brief Handle incoming message from ROS.
    /// \param[in] _msg Quaternion used to set camera orientation.
    void SetCameraGoalAngle(const storm32_gimbal::GimbalOrientationConstPtr &_msg);

  private:
    // Pointer to the robot model.
    physics::ModelPtr model;

    // Pointers for the joints we want to control.
    physics::JointPtr roll_joint;
    physics::JointPtr pitch_joint;

    // For storing and fetching the desired pitch and roll.
    double camera_pitch;
    double camera_roll;

    /*
     * Below are helper functions from a ROS Gazebo plugin tutorial.
     */

     /// \brief ROS helper function that processes messages.
     void QueueThread();

    // Pointer to the update event connection.
    event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport.
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber for the gimbal.
    ros::Subscriber gimbal_subscriber;

    /// \brief A ROS publisher for the gimbal.
    ros::Publisher gimbal_publisher;

    /// \brief A ROS callbackqueue that helps process messages.
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue.
    std::thread rosQueueThread;
  };
}

#endif // __DRONE_GAZEBO_GOONEY_PLUGIN_H_
