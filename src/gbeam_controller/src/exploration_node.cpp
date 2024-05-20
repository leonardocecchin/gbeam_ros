#include "ros/ros.h"
#include "ros/console.h"

#include <math.h>
#include <string>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"

#include <gbeam_library/Vertex.h>

#include "polytope_fcn.h"
#include "graph_fcn.h"
#include "exploration_fcn.h"

#define INF 1000000

ros::Publisher pos_ref_pub;

geometry_msgs::TransformStamped l2g_tf;

gbeam_library::ReachabilityGraph graph;
int N = 0, E = 0;

float rate = 2;
int last_target = -1;
float limit_xi = 0, limit_xs = 0, limit_yi = 0, limit_ys = 0;
float distance_exp = 2;
float mapping_z = 5;
gbeam_library::Vertex last_target_vertex;

void graphCallback(const gbeam_library::ReachabilityGraph::ConstPtr& graph_ptr)
{
  graph = *graph_ptr;
  N = graph.nodes.size();
  E = graph.nodes.size();
}

void computeNewTarget()
{
  geometry_msgs::PoseStamped pos_ref;

  float max_reward = -INF;
  int best_node = 0;
  float dist[N];

  if (last_target < 0)
  {
    ROS_WARN("No target selected yet, selecting first node");
    last_target = 0;
    last_target_vertex = graph.nodes[0];;

    pos_ref.pose.position = vertex2point(last_target_vertex);
    pos_ref.pose.position.z = mapping_z;

    // Fill other properties of pos_ref
    pos_ref.header.stamp = ros::Time::now();
    pos_ref.header.frame_id = "odom";

    pos_ref.pose.orientation.x = 0.0;
    pos_ref.pose.orientation.y = 0.0;
    pos_ref.pose.orientation.z = 0.0;
    pos_ref.pose.orientation.w = 1.0;

    pos_ref_pub.publish(pos_ref);
    return;
  }

  shortestDistances(graph, dist, last_target);

  for(int n=0; n<N; n++)
  {
    // if((graph.nodes[n].x > limit_xi) && (graph.nodes[n].x < limit_xs) && (graph.nodes[n].y > limit_yi) && (graph.nodes[n].y < limit_ys))
    if(graph.nodes[n].is_reachable && n!=last_target)  // choose only reachable targets, different from last target
    {
      float reward = graph.nodes[n].gain / pow(dist[n],distance_exp);
      if(reward > max_reward && n != last_target)
      {
        max_reward = reward;
        best_node = n;
      }
    }
  }
  // Terminate node if all rewards are null
  if (max_reward <= 0)
  {
    ROS_WARN("All rewards are null, terminating node");
    ros::shutdown();
  }

  if (best_node == last_target)
  {
    ROS_WARN("Best node is the same as last target, maintaining last target");
  }
  else
  {
    ROS_INFO("Target node: %d, computing path from %d to %d", best_node, last_target, best_node);
    std::vector<int> path = dijkstra(graph, last_target, best_node);

    if (path.size() > 1)
    {
      std::string path_str;
      for (int i=0; i<path.size(); i++)
        path_str = path_str + std::to_string(path[i]) + "-";
      path_str.pop_back();  // remove last '-'
      ROS_INFO("New path is: %s", path_str.c_str());

      last_target = path[1];
    }
    else
    {
      ROS_WARN("No path found, maintaining last target");
    }
  }
  

  gbeam_library::Vertex vert = applyBoundary(graph.nodes[last_target], limit_xi, limit_xs, limit_yi, limit_ys);
  last_target_vertex = vert;

  pos_ref.pose.position = vertex2point(vert);
  pos_ref.pose.position.z = mapping_z;

  // Fill other properties of pos_ref
  pos_ref.header.stamp = ros::Time::now();
  pos_ref.header.frame_id = "odom";

  pos_ref.pose.orientation.x = 0.0;
  pos_ref.pose.orientation.y = 0.0;
  pos_ref.pose.orientation.z = 0.0;
  pos_ref.pose.orientation.w = 1.0;

  pos_ref_pub.publish(pos_ref);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_expl");
  ros::NodeHandle n;
  ros::param::get("/gbeam_controller/rate", rate);
  ros::Rate loop_rate(rate);

  ros::Subscriber graph_sub = n.subscribe("gbeam/reachability_graph", 1, graphCallback);

  pos_ref_pub = n.advertise<geometry_msgs::PoseStamped>("gbeam/gbeam_pos_ref", 1);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  float reached_tol = 0.5;

  while(n.ok())
  {
	  try
	  {
		  //get transformation to "/odom" from "/base_scan"
		  l2g_tf = tfBuffer.lookupTransform("odom", "base_scan", ros::Time(0));
	  }
	  catch(tf2::TransformException &ex)
	  {
		  ROS_WARN("GBEAM:exploration:lookupTransform: %s", ex.what());
		  ros::Duration(1.0).sleep();
		  continue;
	  }

    // get updated parameters
    ros::param::get("/gbeam_controller/exploration_param/reached_tol", reached_tol);
    ros::param::get("/gbeam_controller/exploration_param/limit_xi", limit_xi);
    ros::param::get("/gbeam_controller/exploration_param/limit_xs", limit_xs);
    ros::param::get("/gbeam_controller/exploration_param/limit_yi", limit_yi);
    ros::param::get("/gbeam_controller/exploration_param/limit_ys", limit_ys);
    ros::param::get("/gbeam_controller/exploration_param/distance_exp", distance_exp);
    ros::param::get("/gbeam_controller/exploration_param/mapping_z", mapping_z);

    //create dummy vertex corresponding to robot position
    gbeam_library::Vertex position;
    position = vert_transform(position, l2g_tf);  // create temporary vertex at robot position
    // check if last_target has been reached
    if(N>0)
    {
      if(last_target < 0)
        computeNewTarget();
      else
        if(dist(last_target_vertex, position) <= reached_tol)
          computeNewTarget();
    }

	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
