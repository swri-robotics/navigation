/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <move_base/MoveBaseConfig.h>
#include <move_base/global_navigator.h>
#include <move_base/local_navigator.h>
#include <move_base/state_machine.h>

namespace move_base
{
//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;



/**
 * @class MoveBase
 * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
 */
class MoveBase
{
public:
  /**
   * @brief  Constructor for the actions
   * @param name The name of the action
   * @param tf A reference to a TransformListener
   */
  MoveBase(tf::TransformListener& tf);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~MoveBase();

private:
  /**
   * @brief  A service call that clears the costmaps of obstacles
   * @param req The service request
   * @param resp The service response
   * @return True if the service call succeeds, false otherwise
   */
  bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  /**
   * @brief  Clears obstacles within a window around the robot
   * @param size_x The x size of the window
   * @param size_y The y size of the window
   */
  void clearCostmapWindows(double size_x, double size_y);

  /**
   * @brief  Reset the state of the move_base action and send a zero velocity command to the base
   */
  //void resetState();

  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

  void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

  bool isQuaternionValid(const geometry_msgs::Quaternion& q);

  geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

  tf::TransformListener& tf_;

  MoveBaseActionServer* as_;

  GlobalNavigator global_nav_;
  LocalNavigator local_nav_;
  boost::shared_ptr<StateMachine> state_machine_;

  tf::Stamped<tf::Pose> global_pose_;
  ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
  ros::Subscriber goal_sub_;

  boost::recursive_mutex configuration_mutex_;
  dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;

  void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);
  bool setup_;
  
  pluginlib::ClassLoader<StateMachine> state_machine_loader_;
};
}
;
#endif

