#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <steering_control/dwa_bicycle.h>
#include "std_msgs/Float64.h"


#include <sstream>

double beta2_dot;
void beta2_dotCallback(const std_msgs::Float64 & beta2_dot_sub)
{
    beta2_dot = beta2_dot_sub.data;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "beta2");

  ros::NodeHandle n;
  double beta2 = 0;
  std_msgs::Float64 beta2_pub;
  double freq = 100;
  double dt = 1/freq;


  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("control/local_planner/beta2", 100);
  ros::Subscriber sub = n.subscribe("control/local_planner/beta2_dot",100, beta2_dotCallback);

  ros::Rate loop_rate(freq);


  while (ros::ok())
  {
    beta2 += dt*beta2_dot;
    beta2_pub.data = beta2;
    chatter_pub.publish(beta2_pub);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
