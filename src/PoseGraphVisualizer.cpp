/**
 * @file PoseGraphVisualizer.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 *
 */

#include "PoseGraphVisualizer.h"

#include <visualization_msgs/Marker.h>

namespace pgv {

// ------------------------------------------------------------------------------
//  Public methods
// ------------------------------------------------------------------------------

PoseGraphVisualizer::PoseGraphVisualizer(const ros::NodeHandle& node_handle)
  : m_node_handle(node_handle),
    m_sub_pose_graph(m_node_handle.subscribe("pose_graph", 1000, &PoseGraphVisualizer::poseGraphCallback, this)),
    m_pub_graph_nodes(m_node_handle.advertise<visualization_msgs::Marker>("nodes", 5)),
    m_pub_graph_edges(m_node_handle.advertise<visualization_msgs::Marker>("edges", 5)) {

    }

PoseGraphVisualizer::~PoseGraphVisualizer() {
  ROS_INFO("shutting down the PoseGraphVisualizer");
}


// ------------------------------------------------------------------------------
// private methods
// ------------------------------------------------------------------------------

void PoseGraphVisualizer::poseGraphCallback(const pose_graph_visualizer::GraphSE3& msg) {

  ROS_DEBUG_STREAM("received graph message");

  //
  // publish the vertices as a list of cubes

  // create a marker
  visualization_msgs::Marker cube_list;
  cube_list.header.frame_id = msg.header.frame_id;
  cube_list.header.stamp = msg.header.stamp;

  /* Cube lists render faster than marker arrays. Caveat is
  that each cube must have the same scale. */
  cube_list.type = visualization_msgs::Marker::CUBE_LIST;

  for (const auto& vertex : msg.vertices) {

    cube_list.points.push_back(vertex.pose.position);
  }

  // publish the cube list
  m_pub_graph_nodes.publish(cube_list);

  //
  // publish the edges as a list of lines
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = msg.header.frame_id;
  line_list.header.stamp = msg.header.stamp;

  /* Line lists render faster than marker arrays, Caveat is
  that each line must have the same scale. */
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  for (const auto& edge : msg.edges) {
    //
    // get the first point of the edge
    auto vertex_from_it = std::find_if(msg.vertices.begin(), msg.vertices.end(), [&] (const pose_graph_visualizer::VertexSE3& vertex) {
      return edge.from == vertex.index;
    });

    if(vertex_from_it == msg.vertices.end()) {
      ROS_WARN_STREAM("Could not find vertex with index " << edge.from);
      return;
    }
    auto vertex_from = *vertex_from_it;

    // get the second point of the edge
    auto vertex_to_it = std::find_if(msg.vertices.begin(), msg.vertices.end(), [&](const pose_graph_visualizer::VertexSE3& vertex) {
      return edge.to == vertex.index;
    });

    if(vertex_to_it == msg.vertices.end()) {
      ROS_WARN_STREAM("Could not find vertex with index " << edge.to);
      return;
    }
    auto vertex_to = *vertex_to_it;

    // add the line between the vertices to the line list
    line_list.points.push_back(vertex_from.pose.position);
    line_list.points.push_back(vertex_to.pose.position);

  }
  // publish the line
  m_pub_graph_edges.publish(line_list);
}
} // namespace pgv