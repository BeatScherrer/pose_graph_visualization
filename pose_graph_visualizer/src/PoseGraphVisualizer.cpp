/**
 * @file PoseGraphVisualizer.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 *
 */

#include "PoseGraphVisualizer.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace pgv {

// ------------------------------------------------------------------------------
//  Public methods
// ------------------------------------------------------------------------------

PoseGraphVisualizer::PoseGraphVisualizer(const ros::NodeHandle& node_handle)
    : m_node_handle(node_handle),
      m_sub_pose_graph(m_node_handle.subscribe(
          "pose_graph_in", 1000, &PoseGraphVisualizer::poseGraphCallback, this)),
      m_pub_pose_graph(
          m_node_handle.advertise<visualization_msgs::MarkerArray>("pose_graph_out", 5)) {
  ROS_INFO("starting the PoseGraphVisualizer...");
}

PoseGraphVisualizer::~PoseGraphVisualizer() {
  ROS_INFO("...shutting down the PoseGraphVisualizer");
}

// ------------------------------------------------------------------------------
// private methods
// ------------------------------------------------------------------------------
// TODO refactor this method into multiple functions
void PoseGraphVisualizer::poseGraphCallback(
    const pose_graph_visualizer_msgs::GraphSE3& msg) {
  ROS_DEBUG_STREAM("received graph message");

  unsigned int number_of_vertex_sets = msg.vertex_arrays.size();
  unsigned int number_of_edge_sets = msg.edge_arrays.size();

  //
  // publish the vertices as a list of cubes

  visualization_msgs::MarkerArray marker_array;

  int i = 0;
  // for each vertex array add a CUBE_LIST marker with different colors.
  for (const auto& node_set : msg.vertex_arrays) {
    const auto& nodes = node_set.vertex_array;

    // create a marker
    visualization_msgs::Marker cube_list;
    cube_list.header.frame_id = msg.header.frame_id;
    cube_list.header.stamp = msg.header.stamp;
    cube_list.pose.orientation.w = 1.0;
    cube_list.pose.orientation.x = 0.0;
    cube_list.pose.orientation.y = 0.0;
    cube_list.pose.orientation.z = 0.0;
    cube_list.ns = "nodes";

    /* Cube lists render faster than marker arrays. Caveat is
    that each cube must have the same scale. */

    cube_list.type = visualization_msgs::Marker::CUBE_LIST;

    // marker properties
    cube_list.scale.x = 0.1;
    cube_list.scale.y = 0.1;
    cube_list.scale.z = 0.1;
    cube_list.color.a = 1.0;
    cube_list.color.r = 0.0;
    cube_list.color.g = 0.0;
    cube_list.color.b = 0.2 + 0.8 / number_of_vertex_sets * i;

    for (const auto& vertex : nodes) {
      cube_list.points.push_back(vertex.pose.position);
    }

    // add cube list to the marker array
    marker_array.markers.push_back(cube_list);

    ++i;
  }

  int j = 0;

  // TODO add different colors
  // for each edge array add a LINE_LIST marker with different colors
  for (const auto& edge_set : msg.edge_arrays) {
    const auto& edges = edge_set.edge_array;
    //
    // publish the edges as a list of lines
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = msg.header.frame_id;
    line_list.header.stamp = msg.header.stamp;

    /* Line lists render faster than marker arrays, Caveat is
    that each line must have the same scale. */
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.02;
    line_list.color.a = 1.0;
    line_list.color.r = 0.2 + 0.8 / number_of_edge_sets * j;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.pose.orientation.w = 1.0;
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;
    line_list.ns = "edges";

    // TODO combine with above to now loop twice over the nodes
    // need to look for points in all the node arrays
    for (const auto& edge : edges) {
      for (const auto& node_set : msg.vertex_arrays) {
        const auto& nodes = node_set.vertex_array;
        //
        // get the first point of the edge
        auto vertex_from_it =
            std::find_if(nodes.begin(), nodes.end(),
                         [&](const pose_graph_visualizer_msgs::VertexSE3& vertex) {
                           return edge.from == vertex.index;
                         });

        if (vertex_from_it == nodes.end()) {
          // ROS_WARN_STREAM("Could not find vertex with index " << edge.from);
          continue;
        }
        auto vertex_from = *vertex_from_it;

        // get the second point of the edge
        auto vertex_to_it =
            std::find_if(nodes.begin(), nodes.end(),
                         [&](const pose_graph_visualizer_msgs::VertexSE3& vertex) {
                           return edge.to == vertex.index;
                         });

        if (vertex_to_it == nodes.end()) {
          // ROS_WARN_STREAM("Could not find vertex with index " << edge.to);
          continue;
        }
        auto vertex_to = *vertex_to_it;

        // add the line between the vertices to the line list
        line_list.points.push_back(vertex_from.pose.position);
        line_list.points.push_back(vertex_to.pose.position);
      }
    }
    // add line list to the marker array
    marker_array.markers.push_back(line_list);

    ++j;
  }

  m_pub_pose_graph.publish(marker_array);
}

}  // namespace pgv