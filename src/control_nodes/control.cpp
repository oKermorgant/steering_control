#include "control.h"
#include <ros/package.h>
#include <filesystem>

void ensureRosNamespace(int& argc, char** argv, std::string ns)
{
  if(ns != "")
    strcpy(argv[0], (std::string("__ns:=") + ns).c_str());
  ros::init(argc, argv, "control");
}

Controller::Controller(): priv("~"), buffer(ros::Duration(10)), tl(buffer)
{

  if(!priv.hasParam("local_costmap"))
  {
    ROS_WARN("No costmap parameters - run this node through a launch file at least once");
    ros::shutdown();
  }

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // get planning service
  plan_srv = nh.serviceClient<nav_msgs::GetPlan>("planner/planner/make_plan");

  // emulate move_base_simple/goal
  goal_sub = nh.subscribe(priv.getNamespace() + "_simple/goal", 10, &Controller::goalCallback, this);

  // emulate move_base/status @ control freq
  status_pub = priv.advertise<actionlib_msgs::GoalStatusArray>("status", 10);
  refresh_still.fromSec(1);
  refresh_moving.fromSec(1./priv.param("controller_frequency", 5.));
  timer = nh.createTimer(refresh_still, [&](auto &){refresh();});

  respawnRobot();
}

void Controller::loadParam(const YAML::Node &node, std::string ns)
{
  static const std::map<std::string, bool> toBool
  {{"true", true}, {"True", true}, {"false", false}, {"False", false}};

  if(node.IsMap())
  {
    for(YAML::const_iterator it = node.begin();
        it != node.end(); it++)
      loadParam(it->second, ns + "/" + it->first.as<std::string>());
  }
  else if(node.IsScalar())
  {
    const auto val(node.as<std::string>());
    if(toBool.find(val) == toBool.end())
      try
    {
      priv.setParam(ns, std::stod(val));
    }
    catch (...)
    {
      priv.setParam(ns, val);
    }
    else
      priv.setParam(ns, toBool.at(val));
  }
}

void Controller::loadParamFile(std::string config_file, std::string ns)
{
  if(package_path == "")
  {
    package_path = ros::package::getPath("steering_control");
  }

  // find this file where it should be
  config_file = package_path + "/param/" + config_file;
  if(std::filesystem::exists(config_file))
  {
    loadParam(YAML::LoadFile(config_file), ns);
  }
  else
  {
    ROS_ERROR("Config file %s does not exist", config_file.c_str());
  }
}

void Controller::respawnRobot() const
{
  const auto x(priv.param("x0", 0.));
  const auto y(priv.param("y0", 0.));
  const auto theta(priv.param("theta0", 0.));

  std::stringstream ss;
  ss << "rosrun map_simulator spawn"
     << " __ns:=/robot "
     << " _x:=" << x
     << " _y:=" << y
     << " _theta:=" << theta
     << " _static_tf_odom:=true"
     << " _force_scanner:=false";
  system(ss.str().c_str());
}

inline void Controller::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
  if(!isReady())
    return;

  // to service
  static nav_msgs::GetPlan plan;

  // get pose of robot in map frame
  convert(buffer.lookupTransform("map", costmap->getBaseFrameID(), ros::Time(0)),
          plan.request.start);
  plan.request.goal = *goal;

  if(!plan_srv.call(plan) || plan.response.plan.poses.size() == 0)
    return;

  if(local_planner->setPlan(plan.response.plan.poses))
  {
    status = Status::MOVING;
    timer.setPeriod(refresh_moving);
  }
}

void Controller::refresh()
{
  if(!isReady())
    return;

  static actionlib_msgs::GoalStatusArray goal_status;
  goal_status.status_list.resize(1);

  cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z =
      cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0;

  if(status == Status::MOVING &&
     (local_planner->isGoalReached() || !local_planner->computeVelocityCommands(cmd_vel)))
  {
    status = Status::DONE;
    timer.setPeriod(refresh_still);
  }

  cmd_vel_pub.publish(cmd_vel);

  goal_status.status_list.back().status =
      status == Status::DONE ? actionlib_msgs::GoalStatus::PENDING : actionlib_msgs::GoalStatus::ACTIVE;

  goal_status.header.stamp = ros::Time::now();
  status_pub.publish(goal_status);
}
