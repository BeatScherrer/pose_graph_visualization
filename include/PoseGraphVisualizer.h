#pragma once

/**
 * @brief Declaration of the PoseGraphVisualizer class.
 */

#include <ros/ros.h>

#include "pose_graph_visualizer/GraphSE3.h"

namespace pgv {

/**
 * @brief Visualizes a pose graph in rviz.
 * Instead of sending individual triangles and lines to rviz,
 * an RViz plugin can be created to visualize a pose graph
 * directly which would be prettier...
 */
class PoseGraphVisualizer
{
public:
  PoseGraphVisualizer(const ros::NodeHandle& node_handle);
  ~PoseGraphVisualizer();

private:
  void poseGraphCallback(const pose_graph_visualizer::GraphSE3& msg);

  ros::NodeHandle m_node_handle;
  ros::Subscriber m_sub_pose_graph;
  ros::Publisher m_pub_graph_nodes;
  ros::Publisher m_pub_graph_edges;

};


} // namespace pgv