#include "control.h"
#include <steering_control/greedy_nowalls.h>

int main(int argc, char* argv[])
{
  ensureRosNamespace(argc, argv);

  Controller control;

  // we use a very local costmap to get a goal position not too far
  control.setLocalParam("local_costmap/width", 1.);
  control.setLocalParam("local_costmap/height", 1.);

  // run stuff, dynamically reload the config file for debug purpose
  control.initLocalPlanner<GreedyNoWalls>("greedy_nowalls.yaml");

  ros::spin();
}
