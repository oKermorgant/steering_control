#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/GetPlan.h>
#include <nav_core/base_local_planner.h>
#include <yaml-cpp/yaml.h>

namespace
{

inline void convert(const geometry_msgs::TransformStamped &tf, geometry_msgs::PoseStamped &pose)
{
  pose.header.frame_id = tf.header.frame_id;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation.w = tf.transform.rotation.w;
  pose.pose.orientation.x = tf.transform.rotation.x;
  pose.pose.orientation.y = tf.transform.rotation.y;
  pose.pose.orientation.z = tf.transform.rotation.z;
}
}

void ensureRosNamespace(int& argc, char **argv, std::string ns = "/robot");

class Controller
{
public:
  Controller();


  template <class LocalPlanner>
  void initLocalPlanner(std::string config_file = "")
  {
    // init costmap after potentially changing params
    costmap = std::make_unique<costmap_2d::Costmap2DROS>("local_costmap", buffer);

    // override params
    if(config_file != "")
      loadParamFile("local_planners/" + config_file, "local_planner");

    // init the local planner
    local_planner = std::make_unique<LocalPlanner>();
    local_planner->initialize("local_planner", &buffer, costmap.get());
  }

  template <class T>
  inline void setLocalParam(std::string name, T value)
  {
    priv.setParam(name, value);
  }

private:

  void respawnRobot() const;

  inline bool isReady() const
  {
    if(!local_planner.get())
    {
      ROS_ERROR("LocalPlanner was not initialized");
      return false;
    }
    return true;
  }

  enum class Status {MOVING, DONE};

  std::string package_path = "";
  void loadParamFile(std::string config_file, std::string ns);
  void loadParam(const YAML::Node &node, std::string ns);

  ros::NodeHandle nh, priv;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tl;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap;

  ros::Publisher cmd_vel_pub;
  geometry_msgs::Twist cmd_vel;
  std::unique_ptr<nav_core::BaseLocalPlanner> local_planner;
  Status status = Status::DONE;

  // emulate planning from topic
  ros::Subscriber goal_sub;
  ros::ServiceClient plan_srv;
  inline void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);

  // emulate move_base goal feedback
  ros::Publisher status_pub;
  void refresh();
  ros::Timer timer;
  ros::Duration refresh_still, refresh_moving;
};

#endif
