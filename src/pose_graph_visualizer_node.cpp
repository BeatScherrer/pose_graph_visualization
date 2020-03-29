/**
 * @file pose_graph_visualizer_node.cpp
 * @author your name (you@domain.com)
 * @brief Visualizes a pose graph published on the given topic.
 * @version 0.1
 * @date 2020-03-28
 *
 */

#include <ros/ros.h>
#include <PoseGraphVisualizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph_visualizer_node");

  ros::NodeHandle node_handle("pose_graph_visualizer");

  pgv::PoseGraphVisualizer pose_graph_visualizer(node_handle);

  return 0;
}


