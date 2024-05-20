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

int num_vertices = 8;
float dist_step = 0.01;
float safe_dist = 3;
float obstacle_d_thr = 0.1;
float start_dist = 0.02;
float update_freq = 2;

sensor_msgs::LaserScan scan;

ros::Publisher free_poly_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr)
{
  scan = *scan_ptr;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poly_gen");
  ros::NodeHandle n;

  ros::Subscriber scan_sub = n.subscribe("scan", 1, scanCallback);
  free_poly_pub = n.advertise<gbeam_library::FreePolygonStamped>("gbeam/free_polytope", 1);

  ros::param::get("/gbeam_controller/rate", update_freq);
  ros::Rate loop_rate(update_freq); // execute at 1 Hz

  while(ros::ok())
  {
    ros::spinOnce();

    int num_measurements = scan.ranges.size();

    if (num_measurements > 0)
    {
      ros::param::get("/gbeam_controller/polytope_generation_param/num_vertices", num_vertices);
      ros::param::get("/gbeam_controller/polytope_generation_param/distance_step", dist_step);
      ros::param::get("/gbeam_controller/polytope_generation_param/start_distance", start_dist);
      ros::param::get("/gbeam_controller/polytope_generation_param/vertex_obstacle_dist", obstacle_d_thr);
      ros::param::get("/gbeam_controller/robot_param/safe_dist", safe_dist);
      float obst_dist_min = 0.1;

      // initialize vector of obstacle points
      sensor_msgs::PointCloud obstacles;
      obstacles.header = scan.header;
      for(int i=0; i<num_measurements; i++)
      {
        float a = scan.angle_min + i*scan.angle_increment;
        float range = scan.ranges[i];
        if ((range >= scan.range_max) || (range < start_dist))
        range = scan.range_max;
        else
        {
          geometry_msgs::Point32 p;
          p.x = range * cos(a);
          p.y = range * sin(a);
          p.z = 0;
          obstacles.points.push_back(p);
        }
      }

      // initialize polygon vertices directions
      geometry_msgs::Vector3 vert_directions[num_vertices];
      float angle_diff_vert = 2*M_PI / num_vertices;
      for(int v=0; v<num_vertices; v++)
      {
        float vert_angle = v*angle_diff_vert + angle_diff_vert/2;
        vert_directions[v].x = cos(vert_angle);
        vert_directions[v].y = sin(vert_angle);
        vert_directions[v].z = 0;
      }

      // initialize polygon
      geometry_msgs::PolygonStamped poly;
      poly.header = scan.header;
      bool move_vertex[num_vertices];
      for(int v=0; v<num_vertices; v++)
      {
        move_vertex[v] = true;
        geometry_msgs::Point32 p;
        p = movePoint(p, vert_directions[v], start_dist);
        poly.polygon.points.push_back(p);
      }

      //increase polygon until reaching obstacles or max_distance
      int num_stopped = 0;
      int iter = 0, max_iter = (scan.range_max - start_dist)/dist_step;
      while (num_stopped < num_vertices && iter++ < max_iter)
      {
        for(int v=0; v<num_vertices; v++)
        {
          // generate convex polytope
          if (move_vertex[v])
          {
            poly.polygon.points[v] = movePoint(poly.polygon.points[v], vert_directions[v], dist_step);

            bool containsObstacles = countInsideConv(poly.polygon, obstacles) > 0;

            if (!isConv(poly.polygon) || containsObstacles)
            { //discard move
              poly.polygon.points[v] = movePoint(poly.polygon.points[v], vert_directions[v], -dist_step);
              move_vertex[v] = false;
              num_stopped++;
            } //else keep it and go on
          }
        }
      }

      //initialize free_poly
      gbeam_library::FreePolygonStamped free_poly;
      free_poly.header = poly.header;

      sensor_msgs::ChannelFloat32 expGainChan, obstacleChan;
      expGainChan.name = "exp_gain";
      obstacleChan.name = "is_obstacle";


      //compute exploration gain, copy values to free_poly, vertices
      for(int v=0; v<num_vertices; v++)
      {
        gbeam_library::Vertex vert_obs;   //vertex against obstacle
        vert_obs.x = poly.polygon.points[v].x;
        vert_obs.y = poly.polygon.points[v].y;
        vert_obs.z = poly.polygon.points[v].z;
        float exp_gain = 0;
        int vert_obstacles = num_measurements/num_vertices;
        for (int o=(v*vert_obstacles); o<((v+1)*vert_obstacles); o++)
        {
          if (scan.ranges[o] >= scan.range_max)
          exp_gain++;
          float step = abs(scan.ranges[o]-scan.ranges[o+1]);
          float tmp = step>scan.range_max ? 0 : step / obst_dist_min;
          exp_gain += tmp>1 ? tmp : 0;
        }
        vert_obs.gain = exp_gain;
        vert_obs.is_visited = false;
        vert_obs.is_reachable = false;
        vert_obs.is_completely_connected = false;

        if (countClose(poly.polygon.points[v], obstacles, obstacle_d_thr)>2)
        {
          vert_obs.is_obstacle = true;
          vert_obs.obstacle_normal = computeNormal(poly.polygon.points[v], obstacles, obstacle_d_thr);
        }
        else
        {
          vert_obs.is_obstacle = false;
        }

        free_poly.polygon.vertices_obstacles.push_back(vert_obs);
      }

      free_poly.polygon = createReachablePolygon(free_poly.polygon, safe_dist);

      //publish on topics
      free_poly_pub.publish(free_poly);
    }

    loop_rate.sleep();
  }


  return 0;
}
