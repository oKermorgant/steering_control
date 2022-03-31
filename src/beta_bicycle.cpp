#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <steering_control/dwa_bicycle.h>
#include "std_msgs/Float64.h"


#include <sstream>

double beta_dot;
void beta_dotCallback(const std_msgs::Float64 & beta_dot_sub)
{
    beta_dot = beta_dot_sub.data;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "beta");

  ros::NodeHandle n;
  double beta = 0;
  std_msgs::Float64 beta_pub;
  double freq = 100;
  double dt = 1/freq;


  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("control/local_planner/beta", 100);
  ros::Subscriber sub = n.subscribe("control/local_planner/beta_dot",100, beta_dotCallback);

  ros::Rate loop_rate(freq);


  while (ros::ok())
  {
    beta += dt*beta_dot;
    beta_pub.data = beta;
    chatter_pub.publish(beta_pub);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
