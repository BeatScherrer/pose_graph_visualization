#pragma once

/**
 * @brief Declaration of the PoseGraphVisualizer class.
 */

#include <ros/ros.h>

#include <pose_graph_visualizer_msgs/GraphSE3.h>

namespace pgv {

/**
 * @brief Visualizes a pose graph in rviz.
 */
class PoseGraphVisualizer {
 public:
  PoseGraphVisualizer(const ros::NodeHandle& node_handle);
  ~PoseGraphVisualizer();

 private:
  /**
   * @brief Callback for the @ref m_sub_pose_graph subscriber.
   *
   * @param msg pose graph message type.
   */
  void poseGraphCallback(const pose_graph_visualizer_msgs::GraphSE3& msg);

  /// ROS node handle.
  ros::NodeHandle m_node_handle;

  /// Pose graph subscriber.
  ros::Subscriber m_sub_pose_graph;

  /// Pose graph publisher.
  ros::Publisher m_pub_pose_graph;
};

}  // namespace pgv