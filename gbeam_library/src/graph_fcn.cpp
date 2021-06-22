#include "graph_fcn.h"

#define INF 100000
// used for min_distance initialization

// compute squared distance between 2 vertices_pointcloud
float distSq(gbeam_library::Vertex v1, gbeam_library::Vertex v2)
{
  return (pow(v1.x-v2.x,2)+pow(v1.y-v2.y,2)+pow(v1.z-v2.z,2));
}

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::Point32 vertex2point32(gbeam_library::Vertex v)
{
  geometry_msgs::Point32 p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::Point vertex2point(gbeam_library::Vertex v)
{
  geometry_msgs::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

// convert FreePolygon to Polygon
geometry_msgs::Polygon freePolyObstacles2poly(gbeam_library::FreePolygon f)
{
  geometry_msgs::Polygon p;
  for (int i = 0; i < f.vertices_obstacles.size(); i++)
    p.points.push_back(vertex2point32(f.vertices_obstacles[i]));
  return p;
}

// convert FreePolygon to Polygon
geometry_msgs::Polygon freePolyReachable2poly(gbeam_library::FreePolygon f)
{
  geometry_msgs::Polygon p;
  for (int i = 0; i < f.vertices_reachable.size(); i++)
    p.points.push_back(vertex2point32(f.vertices_reachable[i]));
  return p;
}

// check if point p is inside the polyarea area
bool isInsideArea(gbeam_library::PolyArea area, geometry_msgs::PointStamped p)
{
  int n = area.polygons.size();
  geometry_msgs::Point32 point;
  for (int i=0; i<n; i++)
    if (isInsideConv(area.polygons[i], point))
      return true;
  return false;
}

// check if vertex v is inside the polyarea area
bool isInsideArea(gbeam_library::PolyArea area, gbeam_library::Vertex v)
{
  geometry_msgs::Point32 p = vertex2point32(v);
  int n = area.polygons.size();
  for (int i=0; i<n; i++)
    if (isInsideConv(area.polygons[i], p))
      return true;
  return false;
}

// compute distance between 2 vertices
float dist(gbeam_library::Vertex v, gbeam_library::Vertex u)
{
  return sqrt(pow(v.x-u.x, 2) + pow(v.y-u.y, 2));
}

float dist(gbeam_library::Vertex v, geometry_msgs::PointStamped p)
{
  return sqrt(pow(v.x-p.point.x, 2) + pow(v.y-p.point.y, 2));
}

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance(gbeam_library::ReachabilityGraph graph, gbeam_library::Vertex v)
{
  float d_min = INF;
  for (int i=0; i<graph.nodes.size(); i++)
  {
    float d = dist(graph.nodes[i], v);
    if (d < d_min)
      d_min = d;
  }
  return d_min;
}

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance_noobstacle(gbeam_library::ReachabilityGraph graph, gbeam_library::Vertex v)
{
  float d_min = INF;
  for (int i=0; i<graph.nodes.size(); i++)
  {
    if(!graph.nodes[i].is_obstacle)
    {
      float d = dist(graph.nodes[i], v);
      if (d < d_min)
      d_min = d;
    }
  }
  return d_min;
}

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance_obstacle(gbeam_library::ReachabilityGraph graph, gbeam_library::Vertex v)
{
  float d_min = INF;
  for (int i=0; i<graph.nodes.size(); i++)
  {
    if(graph.nodes[i].is_obstacle)
    {
      float d = dist(graph.nodes[i], v);
      if (d < d_min)
      d_min = d;
    }
  }
  return d_min;
}

// apply transform tf to vertex v, return resulting vertex
gbeam_library::Vertex vert_transform(gbeam_library::Vertex v, geometry_msgs::TransformStamped tf)
{
  geometry_msgs::PointStamped p_in;
    p_in.point.x = v.x; p_in.point.y = v.y; p_in.point.z = v.z;
  geometry_msgs::PointStamped p_out;
  tf2::doTransform(p_in, p_out, tf);

  geometry_msgs::Vector3Stamped vec_in;
    vec_in.vector = v.obstacle_normal;
  geometry_msgs::Vector3Stamped vec_out;
  tf2::doTransform(vec_in, vec_out, tf);

  v.x = p_out.point.x; v.y = p_out.point.y; v.z = p_out.point.z;
  v.obstacle_normal = vec_out.vector;

  return v;
}

// apply tf to FreePolygon p, return result
gbeam_library::FreePolygon poly_transform(gbeam_library::FreePolygon p, geometry_msgs::TransformStamped tf)
{
  gbeam_library::FreePolygon f;
  for (int i=0; i<p.vertices_obstacles.size(); i++)
  {
    f.vertices_obstacles.push_back(vert_transform(p.vertices_obstacles[i], tf));
  }
  for (int i=0; i<p.vertices_reachable.size(); i++)
  {
    f.vertices_reachable.push_back(vert_transform(p.vertices_reachable[i], tf));
  }
  return f;
}

// check if vertex v is inside polytope polygon
bool isInsideObstacles(gbeam_library::FreePolygon poly, gbeam_library::Vertex v)
{
  return isInsideConv(freePolyObstacles2poly(poly), vertex2point32(v));
}

// check if vertex v is inside polytope polygon
bool isInsideReachable(gbeam_library::FreePolygon poly, gbeam_library::Vertex v)
{
  return isInsideConv(freePolyReachable2poly(poly), vertex2point32(v));
}

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
gbeam_library::Vertex applyBoundary(gbeam_library::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys)
{
  if(v.x < l_xi || v.x > l_xs || v.y < l_yi || v.y > l_ys)  // if the vex is outside exploration boundaries
  {
    v.gain = 0;
    v.is_completely_connected = false;
    v.is_obstacle = true;
    v.is_reachable = false;
    v.is_visited = true;
    if(v.x < l_xi)
    {
      v.obstacle_normal.x = 0, v.obstacle_normal.y = 1;
      v.x = l_xi;
    }
    if(v.x > l_xs)
    {
      v.obstacle_normal.x = 0, v.obstacle_normal.y = -1;
      v.x = l_xs;
    }
    if(v.y < l_yi)
    {
      v.obstacle_normal.x = 1, v.obstacle_normal.y = 0;
      v.y = l_yi;
    }
    if(v.y > l_ys)
    {
      v.obstacle_normal.x = -1, v.obstacle_normal.y = 0;
      v.y = l_ys;
    }
  }
  return v;
}

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
bool isInBoundary(gbeam_library::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys)
{
  return v.x < l_xs && v.x > l_xi && v.y < l_ys && v.y > l_yi;
}

// compute data of edge between vertices vert1 and vert2
gbeam_library::GraphEdge computeEdge(gbeam_library::Vertex vert1, gbeam_library::Vertex vert2, float node_bound_dist)
{
  gbeam_library::GraphEdge edge;
  edge.v1 = vert1.id; edge.v2 = vert2.id;
  edge.length = sqrt(distSq(vert1, vert2));
  edge.direction.x = vert2.x - vert1.x;
  edge.direction.y = vert2.y - vert1.y;
  edge.direction.z = 0;
  float norm = sqrt(pow(edge.direction.x, 2) + pow(edge.direction.y, 2));
  edge.direction.x /= norm;
  edge.direction.y /= norm;
  if (vert1.is_obstacle && vert2.is_obstacle && edge.length<node_bound_dist)
    edge.is_boundary = true;
  else
    edge.is_boundary = false;
  edge.is_walkable = false; //set non walkable as default

  return edge;
}
