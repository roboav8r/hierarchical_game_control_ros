#include "highlevel_planner_node.h"
#include "graph_datatypes.h"

// Get ROS map parameters

// Create CV matrix

// Perform BFS to find neighbors

int main(int argc, char** argv)
{
  std::string param;
  ros::init(argc, argv, "my_node_name");
  ros::NodeHandle nh;

  ROS_INFO("Got parameter : %s", param.c_str());
  std::cout << "Got argc : "<< argc << std::endl;
  std::cout << "Got argv : "<< (*argv) << std::endl;

  
  //...
  ros::spin();
  return 0;
}