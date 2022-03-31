#include "control.h"
#include <steering_control/dwa_bicycle.h>

int main(int argc, char* argv[])
{
  ensureRosNamespace(argc, argv);

  Controller control;

  // for this control we need some robot_radius on the local costmap
  control.setLocalParam("local_costmap/robot_radius", 0.2);

  // run stuff, dynamically reload the config file for debug purpose
  control.initLocalPlanner<DWABicycle>("dwa_bicycle.yaml");

  ros::spin();
}
