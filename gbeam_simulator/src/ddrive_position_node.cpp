#include "ros/ros.h"
#include "ros/console.h"

#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"    //odometry, from topic /odom
#include "geometry_msgs/PoseStamped.h"  //ref_pos, from topic /move_base_simple/goal
#include "geometry_msgs/Twist.h"  //output speed ref, to topic /cmd_vel

#define PI 3.14159265359
#define T_PI 2.0*PI

float limit(float v, float max, float min)
{
  return (v<min) ? min : ((v>max) ? max : v);
}

sensor_msgs::LaserScan scan;
nav_msgs::Odometry robot_odom;
geometry_msgs::PoseStamped target_pos;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr)
{
  scan = *scan_ptr;
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr)
{
  robot_odom = *odom_ptr;
}
void refPosCallback(const geometry_msgs::PoseStamped::ConstPtr& tar_pos_ptr)
{
  target_pos = *tar_pos_ptr;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_contr");
  ros::NodeHandle n;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); //set console to show debug messages by default
  ros::Rate loop_rate(10);

  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);
  ros::Subscriber ref_pos_sub = n.subscribe("/ddrive_pos_in", 1, refPosCallback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  geometry_msgs::Twist cmd_vel;

  float k_rho, k_alpha, alpha_thr, yaw_max, vel_max, rho_thr, reached_tol;

  while (ros::ok())
  {
    cmd_vel = geometry_msgs::Twist();
    n.getParam("/gbeam_controller/ddrive_position_param/k_rho", k_rho);
    n.getParam("/gbeam_controller/ddrive_position_param/k_alpha", k_alpha);
    n.getParam("/gbeam_controller/ddrive_position_param/alpha_thr", alpha_thr);
    n.getParam("/gbeam_controller/ddrive_position_param/rho_thr", rho_thr);
    n.getParam("/gbeam_controller/ddrive_position_param/yaw_max", yaw_max);
    n.getParam("/gbeam_controller/ddrive_position_param/vel_max", vel_max);

      float delta_x = target_pos.pose.position.x - robot_odom.pose.pose.position.x;
      float delta_y = target_pos.pose.position.y - robot_odom.pose.pose.position.y;
      float yaw_part_y = 2 * (robot_odom.pose.pose.orientation.w * robot_odom.pose.pose.orientation.z + robot_odom.pose.pose.orientation.x * robot_odom.pose.pose.orientation.y);
      float yaw_part_x = 1 - 2 * (robot_odom.pose.pose.orientation.y * robot_odom.pose.pose.orientation.y + robot_odom.pose.pose.orientation.z * robot_odom.pose.pose.orientation.z);
      float yaw = atan2(yaw_part_y, yaw_part_x); // yaw of the robot

      float rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2)); // distance from the target
      float dest_ang = atan2(delta_y, delta_x); // direction of target wrt pos
      float alpha = atan2(sin(dest_ang-yaw), cos(dest_ang-yaw)); // angular difference from yaw to target direction

      if (rho < rho_thr)
      {
	      cmd_vel.linear.x = 0;
	      cmd_vel.angular.z = 0;
      }
      else
      {
      		if (alpha < alpha_thr && alpha > -alpha_thr)
      		{
	      		cmd_vel.linear.x = limit(k_rho * rho * cos(alpha), vel_max, 0);
              		cmd_vel.angular.z = limit(k_alpha * alpha, yaw_max, -yaw_max);
      		}
      		else
      		{
	      		cmd_vel.linear.x = 0;
              		cmd_vel.angular.z = limit(k_alpha * alpha, yaw_max, -yaw_max);
      		}
      }

    cmd_vel_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
