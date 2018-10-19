/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Waypoint Robotics.
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
 *   * Neither the name of the the copyright holder nor the names of its
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
 * Author:  Nicolas Varas
 *********************************************************************/

#include "custom_recovery/custom_recovery.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(custom_recovery, CustomRecovery,
  custom_recovery::CustomRecovery, nav_core::RecoveryBehavior)

namespace custom_recovery
{

  CustomRecovery::CustomRecovery()
  : tfListener_(NULL), globalCostmapROS_(NULL), localCostmapROS_(NULL)
  , worldModel_(NULL), initialized_(false)
  {
  }

  CustomRecovery::~CustomRecovery()
  {
    delete worldModel_;
  }

  void CustomRecovery::initialize(std::string name,
    tf::TransformListener* tfListener,
    costmap_2d::Costmap2DROS* globalCostmapROS,
    costmap_2d::Costmap2DROS* localCostmapROS)
  {
    if (initialized_)
    {
      ROS_ERROR("Plugin already initialized. Doing nothing.");
      return;
    }

    // store arguments
    name_ = name;
    tfListener_ = tfListener;
    globalCostmapROS_ = globalCostmapROS;
    localCostmapROS_ = localCostmapROS;

    // load parameters
    ros::NodeHandle pnh("~/" + name);

    pnh.param("recovery_speed", recoverySpeed_, 0.2);
    
    pnh.param<int>("escape_cost_threshold", escapeCostThreshold_, 128);
    pnh.param<double>("timeout", timeout_, 5.0);
    pnh.param<double>("extra_footprint_padding", extraFootprintPadding_, 0.1);
    pnh.param<bool>("display_costs", displayCosts_, false);

    twistPub_ = pnh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // get world model
    worldModel_ = new base_local_planner::CostmapModel(
      *localCostmapROS_->getCostmap());

    // set initialization state
    initialized_ = true;
  }

  void CustomRecovery::runBehavior()
  {
    if (!initialized_)
    {
      ROS_ERROR("Plugin must be initialized before recovery behavior is run!");
      return;
    }

    if(globalCostmapROS_ == NULL || localCostmapROS_ == NULL)
    {
      ROS_ERROR("The costmaps passed to the CustomRecovery object cannot "
        "be NULL. Doing nothing.");
      return;
    }

    ROS_WARN("Custom recovery behavior started.");

    ros::Rate loopRate(5);
    ros::Time time = ros::Time::now();


    // get robot footprint
    std::vector<geometry_msgs::Point> footprint;
    footprint = localCostmapROS_->getRobotFootprint();

    // pad footprint
    costmap_2d::padFootprint(footprint, extraFootprintPadding_);

    // transform robot footprint to oriented footprint
    tf::Stamped<tf::Pose> robotPose;
    localCostmapROS_->getRobotPose(robotPose);
    std::vector<geometry_msgs::Point> orientedFootprint;
    costmap_2d::transformFootprint(robotPose.getOrigin().getX(),
      robotPose.getOrigin().getY(), tf::getYaw(robotPose.getRotation()),
      footprint, orientedFootprint);

    double frontLineCost, rearLineCost, leftLineCost, rightLineCost;

    for (unsigned int i = 0; i < 4; i++)
    {
      for (unsigned int j = 0; j < 4; j++)
      {
        if (footprint[i].x > 0 && footprint[i].y > 0 && footprint[j].x > 0 && footprint[j].y < 0)
          frontLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
        if (footprint[i].x < 0 && footprint[i].y > 0 && footprint[j].x < 0 && footprint[j].y < 0)
          rearLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
        if (footprint[i].x > 0 && footprint[i].y > 0 && footprint[j].x < 0 && footprint[j].y > 0)
          leftLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
        if (footprint[i].x > 0 && footprint[i].y < 0 && footprint[j].x < 0 && footprint[j].y < 0)
          rightLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
      }
    }

    ROS_INFO_STREAM_COND(displayCosts_, "Front side cost: " << frontLineCost);
    ROS_INFO_STREAM_COND(displayCosts_, "Rear side cost: " << rearLineCost);
    ROS_INFO_STREAM_COND(displayCosts_, "Left side cost: " << leftLineCost);
    ROS_INFO_STREAM_COND(displayCosts_, "Right side cost: " << rightLineCost);

    int front = frontLineCost < escapeCostThreshold_;
    int rear = rearLineCost < escapeCostThreshold_;
    int left = leftLineCost < escapeCostThreshold_;
    int right = rightLineCost < escapeCostThreshold_;

    bool escapingForward;
    bool escapingLeft;

    if(frontLineCost < rearLineCost ){
      escapingForward = true;
    }else{
      escapingForward = false;
    }

    if(leftLineCost < rightLineCost ){
      escapingLeft = true;
    }else{
      escapingLeft = false;
    }

    geometry_msgs::Twist cmd;

    if(escapingForward){
      cmd.linear.x = 0.1;
    }else{
      cmd.linear.x = -0.1;
    }

    if(escapingLeft){
      cmd.linear.y = 0.1;
    }else{
      cmd.linear.y = -0.1;
    }

    while (ros::ok() && (ros::Time::now() - time).toSec() < timeout_)
    {
      // get robot footprint
      // std::vector<geometry_msgs::Point> footprint;
      footprint = localCostmapROS_->getRobotFootprint();

      // pad footprint
      costmap_2d::padFootprint(footprint, extraFootprintPadding_);

      // transform robot footprint to oriented footprint
      // tf::Stamped<tf::Pose> robotPose;
      localCostmapROS_->getRobotPose(robotPose);
      // std::vector<geometry_msgs::Point> orientedFootprint;
      costmap_2d::transformFootprint(robotPose.getOrigin().getX(),
        robotPose.getOrigin().getY(), tf::getYaw(robotPose.getRotation()),
        footprint, orientedFootprint);

      // double frontLineCost, rearLineCost, leftLineCost, rightLineCost;

      for (unsigned int i = 0; i < 4; i++)
      {
        for (unsigned int j = 0; j < 4; j++)
        {
          if (footprint[i].x > 0 && footprint[i].y > 0 && footprint[j].x > 0 && footprint[j].y < 0)
            frontLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
          if (footprint[i].x < 0 && footprint[i].y > 0 && footprint[j].x < 0 && footprint[j].y < 0)
            rearLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
          if (footprint[i].x > 0 && footprint[i].y > 0 && footprint[j].x < 0 && footprint[j].y > 0)
            leftLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
          if (footprint[i].x > 0 && footprint[i].y < 0 && footprint[j].x < 0 && footprint[j].y < 0)
            rightLineCost = ceil(lineCost(orientedFootprint[i], orientedFootprint[j]));
        }
      }

      front = frontLineCost < escapeCostThreshold_;
      rear = rearLineCost < escapeCostThreshold_;
      left = leftLineCost < escapeCostThreshold_;
      right = rightLineCost < escapeCostThreshold_;

      if (!front && !rear && !left && !right)  // robot cannot go in any direction
      {
        ROS_FATAL("Unable to recover!");
        break;
      }

      if ((escapingForward && !front) || (!escapingForward && !rear))
      {
        cmd.linear.x = 0.0;
      }

      if ((escapingLeft && !left) || (!escapingLeft && !right))
      {
        cmd.linear.y = 0.0;
      }

      cmd.angular.z = 0.0;

      // publish cmd
      twistPub_.publish(cmd);

      loopRate.sleep();
    }

    if ((ros::Time::now() - time).toSec() > timeout_)
      ROS_WARN("Custom recovery behavior timed out!");

    ROS_WARN("Custom recovery behavior finished.");
  }


  double CustomRecovery::lineCost(geometry_msgs::Point point1,
    geometry_msgs::Point point2)
  {
    unsigned int x[3], y[3];
    localCostmapROS_->getCostmap()->worldToMap(point1.x, point1.y, x[0], y[0]);
    localCostmapROS_->getCostmap()->worldToMap(point2.x, point2.y, x[2], y[2]);

    x[1] = (x[0] + x[2]) / 2;
    y[1] = (y[0] + y[2]) / 2;

    double cost = 0.0;

    for (unsigned int i = 0; i < 3; i++)
      cost += static_cast<unsigned int>(
        localCostmapROS_->getCostmap()->getCost(x[i], y[i])) / 3.0;

    return cost;
  }

}  // namespace custom_recovery
