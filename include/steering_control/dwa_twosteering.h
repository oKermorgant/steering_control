#ifndef STEERING_GREEDY_NOWALLS_H
#define STEERING_GREEDY_NOWALLSL_H

#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <std_msgs/Float64.h>

class DWATwoSteering : public nav_core::BaseLocalPlanner
{
public:

  // mandatory interface from BaseLocalPlanner
  DWATwoSteering() : odom("odom") {}
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  inline bool isGoalReached()
  {
      return goal_reached;
  }
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  //wheel orientation
  double beta1;
  double beta2;


private:

  inline const costmap_2d::Costmap2D & rawCostmap() const
  {
    return *(costmap->getCostmap());
  }

  //geometry
  double Length;

  // config
  double v_gain, w_gain, xy_tolerance, yaw_tolerance, v_max, beta_max, dbeta_max;
  double simtime, time_samples, v_samples, beta_samples;

  // global plan
  bool goal_reached;
  geometry_msgs::PoseStamped local_pose;
  std::vector<geometry_msgs::PoseStamped> global_plan, local_plan;
  bool updateLocalPlan();
  ros::Publisher local_plan_pub, traj_pub, beta1_dot_pub, beta2_dot_pub;
  ros::Subscriber beta1_sub, beta2_sub;

  // geometry
  tf2_ros::Buffer *tf;
  costmap_2d::Costmap2DROS* costmap;
  geometry_msgs::PoseStamped global_pose;
  base_local_planner::OdometryHelperRos odom;
  nav_msgs::Odometry base_odom;

};




#endif
