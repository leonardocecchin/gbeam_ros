#include "ros/ros.h"
#include "ros/console.h"

#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/TransformStamped.h"

#include "gbeam_library/Vertex.h"
#include "gbeam_library/GraphEdge.h"
#include "gbeam_library/PolyArea.h"
#include "gbeam_library/ReachabilityGraph.h"

#include "gbeam_library/setMappingStatus.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

#include "graph_fcn.h"
#include "polytope_fcn.h"


ros::Publisher graph_pub;

gbeam_library::ReachabilityGraph graph;

geometry_msgs::TransformStamped l2g_tf;
tf2_ros::Buffer tfBuffer;

bool mapping_status = false;

float adjacency[10000][10000] = {};

float node_dist_min = 0.2, node_dist_open = 1;
float node_bound_dist = 0.5;
float safe_dist = 0.2, reached_tol = 0.1;
float limit_xi = 0, limit_xs = 0, limit_yi = 0, limit_ys = 0;

float obstacle_margin = 0.05;       // TODO: ADD AS A PARAMETER

bool setStatus(gbeam_library::setMappingStatus::Request &rqt, gbeam_library::setMappingStatus::Response &res)
{
  mapping_status = rqt.request;
  res.response = true;
  return true;
}

void polyCallback(const gbeam_library::FreePolygonStamped::ConstPtr& poly_ptr)
{
  if(!mapping_status)
    return;

  bool is_changed = false;

  //load parameters
  ros::param::get("/gbeam_controller/graph_update_param/node_dist_min", node_dist_min);
  ros::param::get("/gbeam_controller/graph_update_param/node_dist_open", node_dist_open);
  ros::param::get("/gbeam_controller/graph_update_param/node_bound_dist", node_bound_dist);
  ros::param::get("/gbeam_controller/graph_update_param/node_obstacle_margin", obstacle_margin);
  ros::param::get("/gbeam_controller/robot_param/safe_dist", safe_dist);
  // exploration limits
  ros::param::get("/gbeam_controller/exploration_param/limit_xi", limit_xi);
  ros::param::get("/gbeam_controller/exploration_param/limit_xs", limit_xs);
  ros::param::get("/gbeam_controller/exploration_param/limit_yi", limit_yi);
  ros::param::get("/gbeam_controller/exploration_param/limit_ys", limit_ys);
  ros::param::get("/gbeam_controller/exploration_param/reached_tol", reached_tol);
  reached_tol *= 1.5;

  //get transform
  try
  {
    //get transformation from "/odom" to "/base_scan" at timestamp of the polytope (same as the laser scan actually)
    l2g_tf = tfBuffer.lookupTransform("odom", poly_ptr->header.frame_id, poly_ptr->header.stamp);
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("GBEAM:graph_update:lookupTransform: %s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  // ###########################
  // ##### ADD GRAPH NODES #####
  // ###########################

  //add graph vertices if they satisfy condition
  for (int i=0; i<poly_ptr->polygon.vertices_reachable.size(); i++)
  {
    gbeam_library::Vertex vert = poly_ptr->polygon.vertices_reachable[i];  //get vertex from polytope
    vert = vert_transform(vert, l2g_tf); //change coordinates to global position

    float vert_dist = vert_graph_distance_noobstacle(graph, vert);

    // vert = applyBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys);

    if (vert_dist > node_dist_open)
    {
      vert.id = graph.nodes.size();
      vert.is_reachable = true;
      vert.gain ++;
      if (!isInBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys))
      {
        vert.is_reachable = false;
        vert.gain = 0;
      }
      graph.nodes.push_back(vert); //add vertex to the graph
      is_changed = true;
    }
  }
  for (int i=0; i<poly_ptr->polygon.vertices_obstacles.size(); i++)
  {
    gbeam_library::Vertex vert = poly_ptr->polygon.vertices_obstacles[i];  //get vertex from polytope
    vert = vert_transform(vert, l2g_tf); //change coordinates to global position

    vert = moveAway(vert, obstacle_margin);

    float vert_dist = vert_graph_distance_obstacle(graph, vert);

    if ((vert_dist > node_dist_min) && vert.is_obstacle)
    {
      vert.id = graph.nodes.size();
      vert.gain ++;
      if (!isInBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys))
      {
        vert.is_reachable = false;
        vert.gain = 0;
      }
      graph.nodes.push_back(vert); //add vertex to the graph
      is_changed = true;
    }
  }

  // ###########################
  // ##### ADD GRAPH EDGES #####
  // ###########################

  //compute polygon in global coordinates
  gbeam_library::FreePolygon polyGlobal = poly_transform(poly_ptr->polygon, l2g_tf);

  //create vectors with indices of vertices inside polytopes
  std::vector<int> inObstaclesId;
  for (int i=0; i<graph.nodes.size(); i++)
    if (isInsideObstacles(polyGlobal, graph.nodes[i]))
      inObstaclesId.push_back(i);

  // create edges between nodes inside poly_obstacles
  for (int i=0; i<inObstaclesId.size(); i++)
  {
    for (int j=i+1; j<inObstaclesId.size(); j++)
    {
      if (adjacency[inObstaclesId[i]][inObstaclesId[j]] == -1) // if edge not already present
      {
        //then add edge i-j to graph
        gbeam_library::GraphEdge edge = computeEdge(graph.nodes[inObstaclesId[i]], graph.nodes[inObstaclesId[j]], node_bound_dist);
        edge.id = graph.edges.size();
        if(isInsideReachable(polyGlobal, graph.nodes[i]) && isInsideReachable(polyGlobal, graph.nodes[j]))
          edge.is_walkable = true;  // if both vertices are inside reachable poly, then the edge is walkable
        graph.edges.push_back(edge);

        //update adjacency matrix
        adjacency[inObstaclesId[i]][inObstaclesId[j]] = edge.id;
        adjacency[inObstaclesId[j]][inObstaclesId[i]] = edge.id;

        is_changed = true;
      }
      else  // if edge is present, check if it is walkable
      {
        int e = adjacency[inObstaclesId[i]][inObstaclesId[j]];
        if(isInsideReachable(polyGlobal, graph.nodes[graph.edges[e].v1]) && isInsideReachable(polyGlobal, graph.nodes[graph.edges[e].v2]))
          graph.edges[e].is_walkable = true;
      }
    }
  }

  // ########################################
  // ##### UPDATE CONNECTIONS AND GAINS #####
  // ########################################

  // update obstacle nodes connection status, only if something changed
  if(is_changed)
  {
    for(int n=0; n<graph.nodes.size(); n++)
    {
      if(graph.nodes[n].is_obstacle && !graph.nodes[n].is_completely_connected)
      {
        bool connected_left = false, connected_right = false;
        for(int e=0; e<graph.edges.size(); e++)
        {
          if(graph.edges[e].is_boundary && ((graph.edges[e].v1 == n) || (graph.edges[e].v2 == n)))
          {
            // compute angular coefficient of the line containing normal: y=mx
            float m = graph.nodes[n].obstacle_normal.y/graph.nodes[n].obstacle_normal.x;
            // compute value of the inequality mx-y>0, evaluated for edge direction
            float value = m * graph.edges[e].direction.x - graph.edges[e].direction.y;
            if(graph.edges[e].v2 == n)
              value = -value;
            if(value>0)
              connected_right = true;
            else
              connected_left = true;
          }
        }
        if (connected_left && connected_right)
        {
          graph.nodes[n].is_completely_connected = true;
          is_changed = true;
          graph.nodes[n].gain = 0;
        }
      }
    }
  }

  // update exploration gain
  gbeam_library::Vertex position;
  position = vert_transform(position, l2g_tf);  // create temporary vertex at robot position
  for (int i=0; i<graph.nodes.size(); i++)
  {
    // if node is inside reachable area, and gain is > 1, decrease gain to 1
    if (isInsideReachable(polyGlobal, graph.nodes[i]) && graph.nodes[i].gain > 1)
    {
      graph.nodes[i].gain = 1;
      is_changed = true;
    }
    // if node has been visited, set gain to 0
    if (dist(position, graph.nodes[i]) < reached_tol)
    {
      graph.nodes[i].gain = 0;
      graph.nodes[i].is_visited = true;
      is_changed = true;
    }
  }

  // publish graph if some change has occurred
  if(is_changed)
    graph_pub.publish(graph);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_update");
  ros::NodeHandle n;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); //set console to show debug messages by default
  ros::Rate loop_rate(10.0);

  ros::Subscriber poly_sub = n.subscribe("gbeam/free_polytope", 1, polyCallback);

  graph_pub = n.advertise<gbeam_library::ReachabilityGraph>("gbeam/reachability_graph", 1);

  ros::ServiceServer status_server = n.advertiseService("gbeam/set_mapping_status", setStatus);

  tf2_ros::TransformListener tfListener(tfBuffer);

  for(int i=0; i<10000; i++)
    for(int j=0; j<10000; j++)
      adjacency[i][j] = -1;

  ros::spin();

  return 0;
}
