#include "ros/ros.h"
#include "ros/console.h"

#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"

#include <gbeam_library/FreePolygon.h>
#include <gbeam_library/FreePolygonStamped.h>
#include <gbeam_library/Vertex.h>

#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

#include "polytope_fcn.h"


ros::Publisher draw_poly_obs_pub;
ros::Publisher draw_poly_reach_pub;
ros::Publisher vertices_pub;
ros::Publisher poly_normals_pub;


void polyCallback(const gbeam_library::FreePolygonStamped::ConstPtr& poly_ptr)
{
  //initialize polygon, for /poly
  geometry_msgs::PolygonStamped polygon_obs;
    polygon_obs.header = poly_ptr->header;
  geometry_msgs::PolygonStamped polygon_reach;
    polygon_reach.header = poly_ptr->header;

  //initialize vertices, for /poly_vert
  sensor_msgs::PointCloud vertices;
    vertices.header = poly_ptr->header;
  sensor_msgs::ChannelFloat32 expGainChan, obstacleChan;
    expGainChan.name = "exp_gain";
    obstacleChan.name = "is_obstacle";

  //initialize normals, for /poly_normals
  visualization_msgs::Marker normals;
    normals.ns = "graph_drawer", normals.id = 1, normals.type = 5, normals.scale.x = 0.01;
    normals.header = poly_ptr->header;
  std_msgs::ColorRGBA normals_color;
    normals_color.r = 1, normals_color.g = 0.8, normals_color.b = 0.5, normals_color.a = 1;

  //attribute data
  int num_vertices = poly_ptr->polygon.vertices_obstacles.size();
  for(int v=0; v<num_vertices; v++)
  {
    //create point corresponding to vertex obstacle
    geometry_msgs::Point32 point_obs;
    point_obs.x = poly_ptr->polygon.vertices_obstacles[v].x;
    point_obs.y = poly_ptr->polygon.vertices_obstacles[v].y;
    point_obs.z = poly_ptr->polygon.vertices_obstacles[v].z;

    //create point corresponding to vertex reachable
    geometry_msgs::Point32 point_reach;
    point_reach.x = poly_ptr->polygon.vertices_reachable[v].x;
    point_reach.y = poly_ptr->polygon.vertices_reachable[v].y;
    point_reach.z = poly_ptr->polygon.vertices_reachable[v].z;

    //add vertex to polygon
    polygon_obs.polygon.points.push_back(point_obs);
    polygon_reach.polygon.points.push_back(point_reach);

    //add vertex to vertices
    vertices.points.push_back(point_obs);
    expGainChan.values.push_back(poly_ptr->polygon.vertices_obstacles[v].gain);
    obstacleChan.values.push_back(poly_ptr->polygon.vertices_obstacles[v].is_obstacle);

    //add normal to normals if the vertex is obstacle
    if(poly_ptr->polygon.vertices_obstacles[v].is_obstacle)
    {
      geometry_msgs::Point w, z;
      w.x = point_obs.x, w.y = point_obs.y, w.z = point_obs.z;
      z.x = point_obs.x, z.y = point_obs.y, z.z = point_obs.z;
      z.x += 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.x;
      z.y += 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.y;
      w.x -= 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.x;
      w.y -= 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.y;
      normals.points.push_back(w);
      normals.points.push_back(z);
      normals.colors.push_back(normals_color);
      normals.colors.push_back(normals_color);
    }
  }

  //push back channels
  vertices.channels.push_back(expGainChan);
  vertices.channels.push_back(obstacleChan);

  //publish on topics
  draw_poly_obs_pub.publish(polygon_obs);
  draw_poly_reach_pub.publish(polygon_reach);
  vertices_pub.publish(vertices);
  poly_normals_pub.publish(normals);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poly_draw");
  ros::NodeHandle n;
  ros::Rate loop_rate(2);

  ros::Subscriber poly_sub = n.subscribe("gbeam/free_polytope", 1, polyCallback);

  draw_poly_obs_pub = n.advertise<geometry_msgs::PolygonStamped>("gbeam_visualization/poly_obstacles", 1);
  draw_poly_reach_pub = n.advertise<geometry_msgs::PolygonStamped>("gbeam_visualization/poly_reachable", 1);
  vertices_pub = n.advertise<sensor_msgs::PointCloud>("gbeam_visualization/poly_vert", 1);
  poly_normals_pub = n.advertise<visualization_msgs::Marker>("gbeam_visualization/poly_normals", 1);

  ros::spin();

  return 0;
}
