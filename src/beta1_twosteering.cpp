#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <steering_control/dwa_twosteering.h>
#include "std_msgs/Float64.h"


#include <sstream>

double beta2_dot;
void beta1_dotCallback(const std_msgs::Float64 & beta1_dot_sub)
{
    beta2_dot = beta1_dot_sub.data;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "beta1");

  ros::NodeHandle n;
  double beta1 = 0;
  std_msgs::Float64 beta1_pub;
  double freq = 100;
  double dt = 1/freq;


  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("control/local_planner/beta1", 100);
  ros::Subscriber sub = n.subscribe("control/local_planner/beta1_dot",100, beta1_dotCallback);

  ros::Rate loop_rate(freq);


  while (ros::ok())
  {
    beta1 += dt*beta2_dot;
    beta1_pub.data = beta1;
    chatter_pub.publish(beta1_pub);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
