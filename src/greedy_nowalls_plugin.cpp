#include <steering_control/greedy_nowalls.h>
#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>

void GreedyNoWalls::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap)
{
  this->tf = tf;
  this->costmap = costmap;

  // load private parameters
  ros::NodeHandle priv("~" + name);

  v_gain = priv.param("v_gain", 0.5);
  w_gain = priv.param("w_gain", 0.5);
  xy_tolerance = priv.param("xy_tolerance", 0.02);
  yaw_tolerance = priv.param("yaw_tolerance", 0.1);
  v_max = priv.param("v_max", 0.55);
  w_max = priv.param("w_max", 3.);

  local_plan_pub = priv.advertise<nav_msgs::Path>("global_plan", 10);
  traj_pub = priv.advertise<nav_msgs::Path>("local_plan", 10);

  local_pose.header.frame_id = costmap->getBaseFrameID();
  local_pose.pose.orientation.w = 1;
}

bool GreedyNoWalls::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if(!updateLocalPlan())
  {
    // stop
    cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
    return true;
  }

  // get last pose of local_plan
  const auto &goal(local_plan.back().pose);
  local_pose.header.stamp = ros::Time::now();


  base_local_planner::publishPlan({local_pose, local_plan.back()}, traj_pub);

  auto dx(goal.position.x);
  auto dy(goal.position.y);
  double dtheta = 0;

  if(sqrt(dx*dx + dy*dy) < xy_tolerance)
  {
    // almost there, just align
    dtheta = 2*atan2(goal.orientation.z, goal.orientation.w);

    if(std::abs(dtheta) < yaw_tolerance)
      goal_reached = true;
  }
  else
  {
    dtheta = atan2(dy, dx);
  }

  cmd_vel.linear.x = std::clamp(v_gain*dx, -v_max, v_max);
  cmd_vel.linear.y = std::clamp(v_gain*dy, -v_max, v_max);
  cmd_vel.angular.z = std::clamp(w_gain*dtheta, -w_max, w_max);
  return true;
}

bool GreedyNoWalls::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(plan.size())
  {
    goal_reached = false;
    global_plan = plan;
    return true;
  }
  return false;
}

bool GreedyNoWalls::updateLocalPlan()
{
  if(global_plan.empty())
  {
    local_plan.clear();
    return false;
  }
  costmap->getRobotPose(global_pose);
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
       *tf,
       global_plan,
       global_pose,
       *(costmap->getCostmap()),
       costmap->getBaseFrameID(),
       local_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  //base_local_planner::prunePlan(global_pose, local_plan, global_plan);

  base_local_planner::publishPlan(local_plan, local_plan_pub);

  return !local_plan.empty();
}
