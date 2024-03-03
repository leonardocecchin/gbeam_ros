#include <stdio.h>
#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/TransformStamped.h"

#include "gbeam_library/PolyArea.h"
#include "gbeam_library/Vertex.h"
#include "gbeam_library/ReachabilityGraph.h"
#include "gbeam_library/FreePolygonStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "polytope_fcn.h"

// compute squared distance between 2 vertices_pointcloud
float distSq(gbeam_library::Vertex v1, gbeam_library::Vertex v2);

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::Point32 vertex2point32(gbeam_library::Vertex v);

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::Point vertex2point(gbeam_library::Vertex v);

// convert FreePolygon to Polygon
geometry_msgs::Polygon freePolyObstacles2poly(gbeam_library::FreePolygon f);

// convert FreePolygon to Polygon
geometry_msgs::Polygon freePolyReachable2poly(gbeam_library::FreePolygon f);

// check if point p is inside the polyarea area
bool isInsideArea(gbeam_library::PolyArea area, geometry_msgs::PointStamped p);

// check if vertex v is inside the polyarea area
bool isInsideArea(gbeam_library::PolyArea area, gbeam_library::Vertex v);

// compute distance between 2 vertices
float dist(gbeam_library::Vertex v, gbeam_library::Vertex u);

// compute distance between a vertex and a point
float dist(gbeam_library::Vertex v, geometry_msgs::PointStamped p);

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance(gbeam_library::ReachabilityGraph graph, gbeam_library::Vertex v);

// compute minimum distance between vertex v and obstacle graph nodes
float vert_graph_distance_noobstacle(gbeam_library::ReachabilityGraph graph, gbeam_library::Vertex v);

// compute minimum distance between vertex v and reachable graph nodes
float vert_graph_distance_obstacle(gbeam_library::ReachabilityGraph graph, gbeam_library::Vertex v);

// apply transform tf to vertex v, return resulting vertex
gbeam_library::Vertex vert_transform(gbeam_library::Vertex v, geometry_msgs::TransformStamped tf);

// apply tf to FreePolygon p, return result
gbeam_library::FreePolygon poly_transform(gbeam_library::FreePolygon p, geometry_msgs::TransformStamped tf);

// check if vertex v is inside polytope polygon
bool isInsideObstacles(gbeam_library::FreePolygon poly, gbeam_library::Vertex v);

// check if vertex v is inside polytope polygon
bool isInsideReachable(gbeam_library::FreePolygon poly, gbeam_library::Vertex v);

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
gbeam_library::Vertex applyBoundary(gbeam_library::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys);

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
bool isInBoundary(gbeam_library::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys);

// compute data of edge between vertices vert1 and vert2
gbeam_library::GraphEdge computeEdge(gbeam_library::Vertex vert1, gbeam_library::Vertex vert2, float node_bound_dist);
