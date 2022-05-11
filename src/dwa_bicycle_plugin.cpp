#include <steering_control/dwa_bicycle.h>
#include <base_local_planner/goal_functions.h>
#include "std_msgs/Float64.h"
#include <tf2/utils.h>

double orientationFrom(const geometry_msgs::Quaternion &q)
{
  return 2*atan2(q.z, q.w);
}

double sqNorm(double dx,double dy)
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

  // cost tuning
  path_dist_cost = priv.param("path_distance_bias", 32.);
  path_align_cost = priv.param("path_align_bias", 12.);
  goal_cost = priv.param("goal_distance_bias", 20.);
  occdist_scale = priv.param("occdist_scale", .2);

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

  //base_local_planner::publishPlan({local_pose, local_plan.back()}, traj_pub);


  //actual command and the max grade
  auto best_cost(std::numeric_limits<double>::max());
  const auto dt{simtime/(time_samples-1)};

  //table sampling the velocity space (v,beta dot) and associating each pair with a grade
  for(double v = -v_max; v <= v_max; v += v_step)
  {
    for (auto beta_dot = -beta_dot_max; beta_dot <= beta_dot_max; beta_dot += beta_dot_step)
    {
      // start from current position
      auto x{0.};
      auto y{0.};
      auto theta{0.};
      auto current_beta{beta};

      // compute cost for this command
      auto cost{0.};
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
          cost += occdist_scale*(costmap->getCostmap()->getCost(x_map,y_map));
        }

        // trajectory-related cost
        // find closest trajectory point from roboy
        const auto closerToRobot = [x,y](const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
        {
          return sqNorm(p1.pose.position.x-x, p1.pose.position.y-y)
              < sqNorm(p2.pose.position.x-x, p2.pose.position.y-y);
        };
        const auto closest{std::min_element(local_plan.begin(), local_plan.end(), closerToRobot)};
        const auto closest_theta{orientationFrom(closest->pose.orientation)};
        const auto orientation_error{std::abs(fmod(theta-closest_theta+M_PI, 2*M_PI)-M_PI)};
        cost += path_dist_cost*sqrt(sqNorm(closest->pose.position.x-x, closest->pose.position.y-y))
                + path_align_cost*orientation_error;

      }
      // goal-related cost at the end of the simulation
      cost += goal_cost*sqrt(sqNorm(goal.position.x-x, goal.position.y-y));

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
  if(sqrt(sqNorm(goal.position.x, goal.position.y)) < xy_tolerance &&
     std::abs(orientationFrom(goal.orientation)) < yaw_tolerance)
      goal_reached = true;

  cmd_vel.linear.x = cos(beta)*cmd.data[0];
  cmd_vel.angular.z = (sin(beta)/Length)*cmd.data[0];
  cmd_pub.publish(cmd);
  //ros::spinOnce();
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
