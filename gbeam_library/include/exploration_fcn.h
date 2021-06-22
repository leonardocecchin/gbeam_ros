#include <stdio.h>
#include <math.h>
#include <vector>

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
#include "graph_fcn.h"

// find minimum element in array, return its index
int iMin(float arr[], int size);

// find maximum element in array, return its index
int iMax(float arr[], int size);

// return minimum element, from ones with con[i]==true
int iMinCon(float arr[], bool con[], int size);

// return maximum element, from ones with con[i]==true
int iMaxCon(float arr[], bool con[], int size);

//compute distances matrices of graph
void shortestDistances(gbeam_library::ReachabilityGraph graph, float dist[], int start);

// compute shortest path in graph from start to end
// using Dijkstra algorithm
std::vector<int> dijkstra(gbeam_library::ReachabilityGraph graph, int s, int t);

// compute best path from s to t (if t<0 it is ignored)
std::vector<int> bestPath(gbeam_library::ReachabilityGraph graph, int s, int t);
