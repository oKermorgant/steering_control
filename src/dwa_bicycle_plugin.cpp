#include <steering_control/dwa_bicycle.h>
#include <base_local_planner/goal_functions.h>
#include "std_msgs/Float64.h"
#include <tf2/utils.h>

double orientationFrom(const geometry_msgs::Quaternion &q)
{
  return 2*atan2(q.z, q.w);
}

double sqDist(double dx,double dy)
{
  return dx*dx+dy*dy;
}

void DWABicycle::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap)
{
  this->tf = tf;
  this->costmap = costmap;

  // load private parameters
  ros::NodeHandle nh;
  ros::NodeHandle priv("~" + name);

  v_gain = priv.param("v_gain", 0.5);
  w_gain = priv.param("w_gain", 0.5);
  xy_tolerance = priv.param("xy_tolerance", 0.02);
  yaw_tolerance = priv.param("yaw_tolerance", 0.1);
  v_max = priv.param("v_max", 0.55);
  beta_max = priv.param("beta_max", 2.);
  beta_dot_max = priv.param("beta_dot_max", 0.5);

  simtime = priv.param("simtime", 2.);
  time_samples = priv.param("time_samples", 40);
  v_step = 2*v_max / priv.param("v_samples", 10.);
  beta_dot_step = 2*beta_dot_max / priv.param("beta_samples", 21.);

  // length param is parsed and set by fwd_kinematics node
  ros::Rate wait(1.);
  while(!nh.hasParam("fwd_kinematics/L"))
    wait.sleep();
  Length = nh.param("fwd_kinematics/L", 1.);

  local_plan_pub = priv.advertise<nav_msgs::Path>("global_plan", 100);
  traj_pub = priv.advertise<nav_msgs::Path>("local_plan", 100);

  // steering things
  beta_joint = priv.param<std::string>("beta_joint", "steering");
  cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("cmd",100);
  // cmd is of dimension 2
  cmd.data.resize(2, 0.);

  joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 5, [this](sensor_msgs::JointStateConstPtr msg)
  {
              const auto joint = std::find(msg->name.begin(), msg->name.end(), beta_joint);
              if(joint == msg->name.end())
              return;
              beta = msg->position[std::distance(msg->name.begin(), joint)];
});

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

  //base_local_planner::publishPlan({local_pose, local_plan.back()}, traj_pub);

  //parameters to tune, should be ROS parameters
  double alpha_obst = 5.; //effect of the obstacles on the grade
  double alpha_goal = 60.;//effect of proximity to the goal
  double alpha_linear_dist_to_path = 0/time_samples;
  double alpha_angular_dist_to_path = 0/time_samples;

  //actual command and the max grade
  auto best_cost(std::numeric_limits<double>::max());
  const auto dt{simtime/(time_samples-1)};

  //table sampling the velocity space (v,beta dot) and associating each pair with a grade
  for(double v = -v_max; v <= v_max; v += v_step)
  {
    for (auto beta_dot = -beta_dot_max; beta_dot <= beta_dot_max; beta_dot += beta_dot_step)
    {
      // init current position
      auto x{local_pose.pose.position.x};
      auto y{local_pose.pose.position.y};
      auto theta{orientationFrom(local_pose.pose.orientation)};
      auto current_beta{beta};

      // compute cost for this command
      double cost = 0;
      for (int k=0; k<time_samples; k++)
      {
        // move with (v, beta_dot)
        x += v*cos(theta)*cos(current_beta) * dt;
        y += v*sin(theta)*cos(current_beta) * dt;
        theta += v*sin(current_beta)/Length * dt;
        current_beta = std::clamp(current_beta+ beta_dot*dt, -beta_max, beta_max);

        // costmap-related cost
        unsigned int x_map;
        unsigned int y_map;
        if(costmap->getCostmap()->worldToMap(x, y, x_map,y_map))
        {
          cost += alpha_obst*(costmap->getCostmap()->getCost(x_map,y_map));
        }

        // trajectory-related cost
        // find closest point to traj
        const auto distToRobot = [x,y](const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
        {
          return sqDist(p1.pose.position.x-x, p1.pose.position.y-y)
              < sqDist(p2.pose.position.x-x, p2.pose.position.y-y);
        };
        const auto closest{std::min_element(local_plan.begin(), local_plan.end(), distToRobot)};
        const auto closest_theta{orientationFrom(closest->pose.orientation)};
        const auto orientation_error{std::abs(fmod(theta-closest_theta+M_PI, 2*M_PI)-M_PI)};
        cost += alpha_linear_dist_to_path*sqrt(sqDist(closest->pose.position.x-x, closest->pose.position.y-y))
                + alpha_angular_dist_to_path*orientation_error;

      }
      // goal-related cost at the end of the simulation
      cost += alpha_goal*sqrt(sqDist(goal.position.x-x, goal.position.y-y));

      //we check if this is the best trajectory yet
      if(cost < best_cost)
      {
        cmd.data[0] = v;
        cmd.data[1] = beta_dot;
        best_cost = cost;
      }

    }
  }

  //check if the goal is reached
  if(sqrt(sqDist(goal.position.x, goal.position.y)) < xy_tolerance &&
     std::abs(orientationFrom(goal.orientation)) < yaw_tolerance)
      goal_reached = true;

  cmd_vel.linear.x = cos(beta)*cmd.data[0];
  cmd_vel.angular.z = (sin(beta)/Length)*cmd.data[0];
  cmd_pub.publish(cmd);
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
