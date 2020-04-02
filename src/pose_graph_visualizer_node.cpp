/**
 * @file pose_graph_visualizer_node.cpp
 * @brief Visualizes a pose graph published on the given topic.
 */

#include <ros/ros.h>
#include <PoseGraphVisualizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph_visualizer_node");

  ros::NodeHandle node_handle;

  pgv::PoseGraphVisualizer pose_graph_visualizer(node_handle);

  ros::spin();

  return 0;
}


