#include "ros/ros.h"
#include "ros/console.h"

#include <stdio.h>
#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud.h"

#include "gbeam_library/PolyArea.h"
#include "gbeam_library/Vertex.h"
#include "gbeam_library/FreePolygon.h"

// Return the min of 2 numbers
float min(float a, float b);

// Return the max of 2 numbers
float max(float a, float b);
// return dot product between a and b
float dotProd(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);

// return magnitude of vector a
float magnitude(geometry_msgs::Vector3 a);

// return angle of vector b (in 2D!!);
float angle(geometry_msgs::Vector3 a);

// return normalized vector
geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 a);

// return vector a scaled by factor s
geometry_msgs::Vector3 scale(geometry_msgs::Vector3 a, float s);

// Move point p along UNITARY vector v by distance d
geometry_msgs::Point32 movePoint(geometry_msgs::Point32 p, geometry_msgs::Vector3 v, float d);

// Return the distance between point a and b
float distSq(geometry_msgs::Point32 a, geometry_msgs::Point32 b);

// Return the distance between vertices a and b
float distSq(gbeam_library::Vertex a, gbeam_library::Vertex b);

// Given three colinear points p, q, r, the function checks if q lies between p and r
bool onSegment(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r);

// Find orientation of ordered triplet (p, q, r)
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r);

// Check intersection between segment 'p1q1' and 'p2q2'
bool doIntersect(geometry_msgs::Point32 p1, geometry_msgs::Point32 q1, geometry_msgs::Point32 p2, geometry_msgs::Point32 q2);

// Check if p lies inside the polygon poly
bool isInside(geometry_msgs::Polygon poly, geometry_msgs::Point32 p);

// check if p lies inside the convex polygon poly
bool isInsideConv(geometry_msgs::Polygon poly, geometry_msgs::Point32 p);

// check convexity of poly, true if convex
bool isConv(geometry_msgs::Polygon poly);

// count the number of points from query that lay inside poly
int countInside(geometry_msgs::Polygon poly, sensor_msgs::PointCloud query);

// count the number of points of query inside convex polygon poly
int countInsideConv(geometry_msgs::Polygon poly, sensor_msgs::PointCloud query);

// count points in query closer than d_thr to point p
int countClose(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr);

// compute angle of the normal to points in query close to p
geometry_msgs::Vector3 computeNormal(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr);

// create vertices_reachable by offsetting the polygon formed by vertices_obstacles by a distance d
gbeam_library::FreePolygon createReachablePolygon(gbeam_library::FreePolygon in, float d);

// move vertex v away from obstacle (along normal direction), only if node is obstacle
gbeam_library::Vertex moveAway(gbeam_library::Vertex v, float d);
