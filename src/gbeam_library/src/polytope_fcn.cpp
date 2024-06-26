#include "polytope_fcn.h"

#define INF 100000
// infinite for raytracing

// Return the min of 2 numbers
float min(float a, float b)
{
  return a<b ? a : b;
}

// Return the max of 2 numbers
float max(float a, float b)
{
  return a>b ? a : b;
}

// return dot product between a and b
float dotProd(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
	return a.x*b.x + a.y*b.y;// + a.z*b.z;
}

// return magnitude of vector a
float magnitude(geometry_msgs::Vector3 a)
{
	return sqrt(pow(a.x, 2) + pow(a.y, 2));// + pow(a.z, 2));
}

// return angle of vector b (in 2D!!)
float angle(geometry_msgs::Vector3 a)
{
	return atan2(a.y, a.x);
}

// return normalized vector
geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 a)
{
	geometry_msgs::Vector3 b;
	float m = magnitude(a);
	b.x = a.x/m;
	b.y = a.y/m;
	b.z = a.z/m;
	return b;
}

// return vector a scaled by factor s
geometry_msgs::Vector3 scale(geometry_msgs::Vector3 a, float s)
{
	geometry_msgs::Vector3 b;
	b.x = a.x*s;
	b.y = a.y*s;
	b.z = a.z*s;
	return b;
}

// Move point p along UNITARY vector v by distance d
geometry_msgs::Point32 movePoint(geometry_msgs::Point32 p, geometry_msgs::Vector3 v, float d)
{
  p.x += v.x * d;
  p.y += v.y * d;
  p.z += v.z * d;

  return p;
}

// Return the distance between point a and b
float distSq(geometry_msgs::Point32 a, geometry_msgs::Point32 b)
{
  return (pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}
// Return the distance between vertices a and b
float distSq(gbeam_library::Vertex a, gbeam_library::Vertex b)
{
  return (pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}
// Given three colinear points p, q, r, the function checks if q lies between p and r
bool onSegment(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}
// Find orientation of ordered triplet (p, q, r)
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r)
{
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    return (val > 0) ? 1 : ((val<0) ? 2 : 0);
}
// Check intersection between segment 'p1q1' and 'p2q2'
bool doIntersect(geometry_msgs::Point32 p1, geometry_msgs::Point32 q1, geometry_msgs::Point32 p2, geometry_msgs::Point32 q2)
{
    // compute the orientation of the 4 triplets
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) return true;
    // Special Cases
    // p1, q1, p2 colinear and p2 lies on p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    // p1, q1, q2 colinear and q2 lies on p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    // p2, q2, p1 colinear and p1 lies on p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    // p2, q2, q1 colinear and q1 lies on p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // no intersection
}
// Check if p lies inside the polygon poly
bool isInside(geometry_msgs::Polygon poly, geometry_msgs::Point32 p)
{
    int n = poly.points.size();
    // less than 3 vertices --> no polygon
    if (n < 3)  return false;

    // Create a point for line segment from p to infinite
    geometry_msgs::Point32 extreme;
    extreme.x = INF;
    extreme.y = p.y;
    extreme.z = 0;

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(poly.points[i], poly.points[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(poly.points[i], p, poly.points[next]) == 0)
              return onSegment(poly.points[i], p, poly.points[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}
// check if p lies inside the convex polygon poly
bool isInsideConv(geometry_msgs::Polygon poly, geometry_msgs::Point32 p)
{
  if (!isConv(poly))
    return false;
  int n = poly.points.size();
  // less than 3 vertices --> no polygon
  if (n < 3)
    return false;

  bool inCW = true, inCCW = true; //check for both CW and CCW orientation of vertices
  int i = 0;
  do
  {
      int next = (i+1)%n;
      if (orientation(poly.points[i], poly.points[next], p) == 2) // if the triplet is CCW oriented, not in CW polytope
        inCW = false;
      else
        if (orientation(poly.points[i], poly.points[next], p) == 1) // if the triplet is CW oriented, not in CCW polytope
          inCCW = false;

      i = next;
  } while (i != 0);
  return inCW || inCCW;
}
// check convexity of poly, true if convex
bool isConv(geometry_msgs::Polygon poly)
{
  int n = poly.points.size();
  // less than 3 vertices --> no polygon
  if (n < 3)  return false;

  int i = 0;
  do
  {
      int next = (i+1)%n, nnext = (i+2)%n;
      if (orientation(poly.points[i], poly.points[next], poly.points[nnext]) == 1) return false;
      i = next;
  } while (i != 0);
  return true;
}
// count the number of points from query that lay inside poly
int countInside(geometry_msgs::Polygon poly, sensor_msgs::PointCloud query)
{
  int count = 0;
  for(int i=0; i<query.points.size(); i++)
    if (isInside(poly, query.points[i]))
      count++;
  return count;
}
// count the number of points of query inside convex polygon poly
int countInsideConv(geometry_msgs::Polygon poly, sensor_msgs::PointCloud query)
{
  int count = 0;
  for(int i=0; i<query.points.size(); i++)
    if (isInsideConv(poly, query.points[i]))
      count++;
  return count;
}

// count points in query closer than d_thr to point p
int countClose(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr)
{
  int count = 0;
  float d_thr_sq = pow(d_thr,2);
  for (int i=0; i<query.points.size(); i++)
    if (distSq(p, query.points[i]) < d_thr_sq)
      count ++;
  return count;
}

// compute obstacle normal directed away from obstacle, with norm = 1
geometry_msgs::Vector3 computeNormal(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr)
{
  sensor_msgs::PointCloud closePoints;
  float d_thr_sq = pow(d_thr,2);
  float n = 0;
  float xc = 0, yc = 0;
  for (int i=0; i<query.points.size(); i++)
    if (distSq(p, query.points[i]) < d_thr_sq)
    {
      n ++;
      xc += query.points[i].x;
      yc += query.points[i].y;
      closePoints.points.push_back(query.points[i]);
    }
  xc = xc / n;
  yc = yc / n;

  float a=0, b=0, c=0, x=0, y=0;
  for (int i=0; i<n; i++)
  {
    x = closePoints.points[i].x - xc;
    y = closePoints.points[i].y - yc;
    a += x*y;
    b += pow(y,2)-pow(x,2);
    c -= x*y;
  }
  float t = ( -b + sqrt(pow(b,2)-4*a*c) ) / (2*a);

  geometry_msgs::Vector3 v;
  v.z =0;
  v.x = sqrt(1/(1+pow(t,2)));
  v.y = -t*v.x;

  // check normal orientation wrt vector between first and last point (q)
  geometry_msgs::Point32 p_a, p_b;
  p_a = closePoints.points[0];
  p_b = closePoints.points[n-1];
  geometry_msgs::Vector3 q;
  q.x = p_b.x - p_a.x;
  q.y = p_b.y - p_a.y;
  float orientation = v.x*q.y - v.y*q.x;

  if(orientation < 0)
  {
    v.x = -v.x;
    v.y = -v.y;
  }

  return v; // check sign
}

// create vertices_reachable by offsetting the polygon formed by vertices_obstacles by a distance d
gbeam_library::FreePolygon createReachablePolygon(gbeam_library::FreePolygon in, float d)
{
  // prepare output polygon, and clear vertices_reachable
  gbeam_library::FreePolygon poly = in;
  poly.vertices_reachable.erase(poly.vertices_reachable.begin(), poly.vertices_reachable.end());

  // v3 <--n2--> v2 <--n1--> v1
  int num_vert = in.vertices_obstacles.size(), j;
  float nx[num_vert], ny[num_vert], nL, bx, by, bL, l;

  for(int i=0; i<num_vert; i++) //compute polygon edges normals
  {
    // normal i is computed from vertex i to vertex i+1
    j = (i+1)%num_vert; //index off successive vertex
    nx[i] = poly.vertices_obstacles[i].y - poly.vertices_obstacles[j].y;
    ny[i] = poly.vertices_obstacles[j].x - poly.vertices_obstacles[i].x;
    // normalize normals
    nL = sqrt(pow(nx[i], 2)+pow(ny[i], 2));
    nx[i] /= nL;
    ny[i] /= nL;
  }

  for(int i=0; i<num_vert; i++) //move vertices
  {
    // vertex i is moved based on normal i and normal i-1
    j = (i-1)%num_vert; //index of previous normal
    j = (num_vert + ((i-1)%num_vert)) % num_vert;
    // compute bisector vector components
    bx = nx[i]+nx[j];
    by = ny[i]+ny[j];
    // normalize bisector
    bL = sqrt(pow(bx, 2)+pow(by, 2));
    bx /= bL;
    by /= bL;

    // compute vertex movement distance l
    l = d / sqrt(1 + (nx[i]*nx[j] + ny[i]*ny[j])) * sqrt(2);    // srqt(2) scale to get right offset
    // copy vertex and move it by l in direction b
    poly.vertices_reachable.push_back(poly.vertices_obstacles[i]);
    poly.vertices_reachable[i].x += l*bx;
    poly.vertices_reachable[i].y += l*by;
    poly.vertices_reachable[i].is_reachable = true;
    poly.vertices_reachable[i].is_obstacle = false;
    poly.vertices_reachable[i].is_completely_connected = false;
  }

  return poly;
}

// move vertex v away from obstacle (along normal direction), only if node is obstacle
gbeam_library::Vertex moveAway(gbeam_library::Vertex v, float d)
{
  if(!v.is_obstacle)  // if not an obstacle do nothing
    return v;

  // normalize obstacle_normal
  // float norm = sqrt(pow(v.obstacle_normal.x, 2) + pow(v.obstacle_normal.y, 2));
  // v.obstacle_normal.x /= norm;
  // v.obstacle_normal.y /= norm;

  v.x -= v.obstacle_normal.x * d;
  v.y -= v.obstacle_normal.y * d;

  return v;
}
