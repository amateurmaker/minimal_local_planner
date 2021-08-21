
#ifndef MINIMAL_LOCAL_PLANNER_ROS_H_
#define MINIMAL_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

// there are some cool features in goal_functions
#include <base_local_planner/goal_functions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>

namespace minimal_local_planner{

  class MinimalPlannerROS : public nav_core::BaseLocalPlanner{

    public:

      MinimalPlannerROS();


      MinimalPlannerROS(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);


      ~MinimalPlannerROS();

      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;  
      tf2_ros::Buffer* tf_;
      bool initialised_;
      ros::Time begin; 
  };
};
#endif