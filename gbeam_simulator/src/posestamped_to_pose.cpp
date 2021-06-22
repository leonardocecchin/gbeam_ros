#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/Pose.h"

ros::Publisher out_pos_pub;

void refPosCallback(const geometry_msgs::PoseStamped::ConstPtr& tar_pos_ptr)
{
	out_pos_pub.publish(tar_pos_ptr->pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_contr");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Subscriber ref_pos_sub = n.subscribe("/move_base_simple/goal", 1, refPosCallback);
  out_pos_pub = n.advertise<geometry_msgs::Pose>("/goal", 1);

  ros::spin();

  return 0;
}
