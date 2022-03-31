#include <steering_control/dwa_bicycle.h>
#include <base_local_planner/goal_functions.h>
#include "std_msgs/Float64.h"
#include <tf2/utils.h>


double beta_global;
void beta_callback(const std_msgs::Float64 & beta_sub)
{
    beta_global = beta_sub.data;
};


void DWABicycle::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap)
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
  beta_max = priv.param("beta_max", 2.);
  dbeta_max = priv.param("beta_max", 0.5);

  simtime = priv.param("simtime", 2.);
  time_samples = priv.param("time_samples", 40);
  v_samples = priv.param("v_samples", 10);
  beta_samples = priv.param("beta_samples", 21);

  Length = priv.param("length", 0.15);

  local_plan_pub = priv.advertise<nav_msgs::Path>("global_plan", 100);
  traj_pub = priv.advertise<nav_msgs::Path>("local_plan", 100);
  beta_dot_pub = priv.advertise<std_msgs::Float64>("beta_dot",100);
  beta_sub = priv.subscribe("beta",100,beta_callback);

  local_pose.header.frame_id = costmap->getBaseFrameID();
  local_pose.pose.orientation.w = 1;
}

bool DWABicycle::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
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
  beta = beta_global;


  //base_local_planner::publishPlan({local_pose, local_plan.back()}, traj_pub);

  auto delta_x(goal.position.x);
  auto delta_y(goal.position.y);
  double delta_theta = 0;

  //parameters to tune
  double alpha_obst = 5.; //effect of the obstacles on the grade
  double alpha_goal = 60.;//effect of proximity to the goal
  double alpha_linear_dist_to_path = 0/time_samples;
  double alpha_angular_dist_to_path = 0/time_samples;

  //actual command and the max grade
  double v_command = 1.;
  double dbeta_command = 0.;
  auto min_grade(std::numeric_limits<double>::max());


  //table sampling the velocity space (v,beta) and associating each pair with a grade


  for (int i=0 ; i<v_samples; i++)
  {
      for (int j=0 ; j<beta_samples; j++)
      {
          double v = 0 + i*v_max/(v_samples-1);
          double dbeta = -dbeta_max + j*2*dbeta_max/(beta_samples-1);
          geometry_msgs::PoseStamped current_pose = local_pose;
          double dt = simtime/(time_samples-1);
          double current_beta = beta;
          double current_theta;
          //grade for the trajectory
          double grade = 0;
          for (int k=0; k<time_samples; k++){
              current_theta = 2*atan2(sqrt(pow(current_pose.pose.orientation.x,2)+pow(current_pose.pose.orientation.y,2)
                                           +pow(current_pose.pose.orientation.z,2)), current_pose.pose.orientation.w);
              //we then increment all the variables debscribing the state of the robot after a time dt.
              current_pose.pose.position.x += dt*v*(cos(current_theta)*cos(current_beta)-
                                                    0.5*sin(current_theta)*sin(current_beta));
              current_pose.pose.position.y += dt*v*(sin(current_theta)*cos(current_beta)+
                                                    0.5*cos(current_theta)*sin(current_beta));
              current_theta += dt*v*sin(current_beta)/Length;
              current_pose.pose.orientation.x = 0;
              current_pose.pose.orientation.y = 0;
              current_pose.pose.orientation.z = sin(current_theta/2);
              current_pose.pose.orientation.w = cos(current_theta/2);
              current_beta = std::clamp(current_beta+dt*dbeta,-beta_max, beta_max);
              unsigned int x_map;
              unsigned int y_map;
              if(costmap->getCostmap()->worldToMap(current_pose.pose.position.x, current_pose.pose.position.y, x_map,y_map))
              {
                  grade += alpha_obst*(costmap->getCostmap()->getCost(x_map,y_map));
              }

              double dist_to_traj = std::numeric_limits<double>::max();
              double ang_dist_to_traj = std::numeric_limits<double>::max();
              double current_dist_to_traj;
              for(geometry_msgs::PoseStamped pose_traj : local_plan)
              {
                  current_dist_to_traj = sqrt(pow(pose_traj.pose.position.x-current_pose.pose.position.x,2)
                                      +pow(pose_traj.pose.position.y-current_pose.pose.position.y,2));
                  if(current_dist_to_traj < dist_to_traj)
                  {
                      dist_to_traj = current_dist_to_traj;
                      ang_dist_to_traj = abs(2*atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)
                                             -2*atan2(pose_traj.pose.orientation.z, pose_traj.pose.orientation.w));
                  }
              }
              grade += alpha_linear_dist_to_path*dist_to_traj + alpha_angular_dist_to_path*ang_dist_to_traj;
          }
          grade += alpha_goal*(pow(goal.position.x-current_pose.pose.position.x,2)
                               +pow(goal.position.y-current_pose.pose.position.y,2)
                               +pow(goal.orientation.w-current_pose.pose.orientation.w,2));

          //we check if this is the best trajectory yet
          if(grade < min_grade){
              v_command = v;
              dbeta_command = dbeta;
              min_grade = grade;

          }
      }
  }



  //check if the goal is reached
  if(sqrt(delta_x*delta_x + delta_y*delta_y) < xy_tolerance)
  {
    // almost there, just align
    delta_theta = 2*atan2(goal.orientation.z, goal.orientation.w);

    if(std::abs(delta_theta) < yaw_tolerance)
      goal_reached = true;
  }

  auto theta(cmd_vel.angular.z);
  cmd_vel.linear.x = (cos(theta)*cos(beta)-sin(theta)*sin(beta)/2)*v_command;
  cmd_vel.linear.y = (sin(theta)*cos(beta)+cos(theta)*sin(beta)/2)*v_command;
  cmd_vel.angular.z = (sin(beta)/Length)*v_command;
  std_msgs::Float64 beta_dot;
  beta_dot.data = dbeta_command;
  beta_dot_pub.publish(beta_dot);
  ros::spinOnce();
  return true;
}

bool DWABicycle::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(plan.size())
  {
    goal_reached = false;
    global_plan = plan;
    return true;
  }
  return false;
}

bool DWABicycle::updateLocalPlan()
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
