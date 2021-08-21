#include "minimal_local_planner/minimal_local_planner.h"

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(minimal_local_planner::MinimalPlannerROS, nav_core::BaseLocalPlanner)

namespace minimal_local_planner
{

	MinimalPlannerROS::MinimalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialised_(false) {}

	MinimalPlannerROS::MinimalPlannerROS(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
		: costmap_ros_(NULL), tf_(NULL), initialised_(false)
	{
		// initialize planner
		initialize(name, tf, costmap_ros);
	}

	MinimalPlannerROS::~MinimalPlannerROS() {}

	void MinimalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		begin = ros::Time::now(); 

		// check if the plugin is already initialized
		if (!initialised_)
		{
			// copy adress of costmap and Transform Listener (handed over from move_base)
			costmap_ros_ = costmap_ros;
			tf_ = tf;

			// set initialized flag
			initialised_ = true;

			// this is only here to make this process visible in the rxlogger right from the start
			ROS_DEBUG("Simple Local Planner plugin initialized.");
		}
		else
		{
			ROS_WARN("This planner has already been initialized, doing nothing.");
		}
	}

	bool MinimalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
	{
		// check if plugin initialized
		if (!initialised_)
			return false;

		return true;
	}

	bool MinimalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
	{

		if (!initialised_)
			return false;

		// since this is a minimal local planner, don't move
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z = 0.0;

		return true;
	}

	bool MinimalPlannerROS::isGoalReached()
	{
		if (!initialised_)
			return false;

		if (ros::Time::now().toSec() - begin.toSec() > 25.0)
		{
			ROS_INFO("Reached goal"); 
			return true; 
		}
		else 
			ROS_INFO_THROTTLE(5, "It takes time to reach the goal"); 

		// goal is never reached in the minimal local planner
		return false;
	}
}