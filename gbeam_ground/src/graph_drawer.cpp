#include "ros/ros.h"
#include "ros/console.h"

#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"

#include "gbeam_library/Vertex.h"
#include "gbeam_library/GraphEdge.h"
#include "gbeam_library/PolyArea.h"
#include "gbeam_library/ReachabilityGraph.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

#include "graph_fcn.h"


ros::Publisher graph_nodes_pub;
ros::Publisher graph_normals_pub;
ros::Publisher graph_edges_pub;

float scaling = 1;

void graphCallback(const gbeam_library::ReachabilityGraph::ConstPtr& graph_ptr)
{
  ros::param::get("/gbeam_ground/graphics_scaling", scaling);

  // define colors
  std_msgs::ColorRGBA boundary_color;
    boundary_color.r = 1, boundary_color.g = 0.8, boundary_color.b = 0.5, boundary_color.a = 1;
  std_msgs::ColorRGBA inside_color;
    inside_color.r = 0, inside_color.g = 0.6, inside_color.b = 0.5, inside_color.a = 0.15;
    boundary_color.r = 1, boundary_color.g = 0.8, boundary_color.b = 0.5, boundary_color.a = 1;
  std_msgs::ColorRGBA walkable_color;
    walkable_color.r = 1, walkable_color.g = 0.1, walkable_color.b = 0.8, walkable_color.a = 0.15;
  std_msgs::ColorRGBA normals_color;
    normals_color.r = 0.6, normals_color.g = 0.3, normals_color.b = 0.6, normals_color.a = 1;

  float normal_length = 0.2 * scaling;

  //initialize node_points for /graph_nodes
  sensor_msgs::PointCloud node_points;
  sensor_msgs::ChannelFloat32 expGainChan, obstacleChan, connectedChan;
    expGainChan.name = "exploration_gain";
    obstacleChan.name = "is_obstacle";
    connectedChan.name = "is_compl_connected";

  //initialize normals, for /graph_nodes_normals
  visualization_msgs::Marker nodes_normals;
    nodes_normals.ns = "graph_drawer", nodes_normals.id = 1, nodes_normals.type = 5, nodes_normals.scale.x = 0.005 * scaling;

  //initialize edge_markers for /graph_edges
  visualization_msgs::Marker edges_markers;
    edges_markers.ns = "graph_drawer", edges_markers.id = 1, edges_markers.type = 5, edges_markers.scale.x = 0.005 * scaling;
    // edges_markers.pose   could be initialized, actually not needed, ony gives a warning

  //add nodes and nodes normals
  for(int n=0; n<graph_ptr->nodes.size(); n++)
  {
    geometry_msgs::Point32 point;
    point.x = graph_ptr->nodes[n].x;
    point.y = graph_ptr->nodes[n].y;
    point.z = graph_ptr->nodes[n].z;
    node_points.points.push_back(point);
    expGainChan.values.push_back(graph_ptr->nodes[n].gain);
    obstacleChan.values.push_back(graph_ptr->nodes[n].is_obstacle);
    connectedChan.values.push_back(graph_ptr->nodes[n].is_completely_connected);

    if(graph_ptr->nodes[n].is_obstacle)
    {
      geometry_msgs::Point w, z;
      w.x = graph_ptr->nodes[n].x, w.y = graph_ptr->nodes[n].y, w.z = graph_ptr->nodes[n].z;
      z.x = graph_ptr->nodes[n].x, z.y = graph_ptr->nodes[n].y, z.z = graph_ptr->nodes[n].z;
      w.x += 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.x;
      w.y += 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.y;
      z.x -= 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.x;
      z.y -= 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.y;
      nodes_normals.points.push_back(w);
      nodes_normals.points.push_back(z);
      nodes_normals.colors.push_back(normals_color);
      nodes_normals.colors.push_back(normals_color);
    }
  }

  //add edges
  for(int e=0; e<graph_ptr->edges.size(); e++)
  {
    edges_markers.points.push_back(vertex2point(graph_ptr->nodes[graph_ptr->edges[e].v1]));
    edges_markers.points.push_back(vertex2point(graph_ptr->nodes[graph_ptr->edges[e].v2]));
    if (graph_ptr->edges[e].is_boundary)
      {
        edges_markers.colors.push_back(boundary_color);
        edges_markers.colors.push_back(boundary_color);
      }
    else
      {
        if (graph_ptr->edges[e].is_walkable)
        {
          edges_markers.colors.push_back(walkable_color);
          edges_markers.colors.push_back(walkable_color);
        }
        else
        {
          edges_markers.colors.push_back(inside_color);
          edges_markers.colors.push_back(inside_color);
        }
      }
  }
  node_points.channels.push_back(expGainChan);
  node_points.channels.push_back(obstacleChan);
  node_points.channels.push_back(connectedChan);

  node_points.header.frame_id = "odom";
  edges_markers.header.frame_id = "odom";
  nodes_normals.header.frame_id = "odom";

  graph_nodes_pub.publish(node_points);
  graph_normals_pub.publish(nodes_normals);
  graph_edges_pub.publish(edges_markers);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_draw");
  ros::NodeHandle n;
  ros::Rate loop_rate(2);

  ros::Subscriber graph_sub = n.subscribe("gbeam/reachability_graph", 1, graphCallback);

  graph_nodes_pub = n.advertise<sensor_msgs::PointCloud>("gbeam_visualization/graph_nodes", 1);
  graph_normals_pub = n.advertise<visualization_msgs::Marker>("gbeam_visualization/graph_nodes_normals", 1);
  graph_edges_pub = n.advertise<visualization_msgs::Marker>("gbeam_visualization/graph_edges", 1);

  ros::spin();

  return 0;
}
