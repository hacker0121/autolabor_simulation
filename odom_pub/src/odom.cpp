#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

float linX, angZ;

void chatterCallback(const geometry_msgs::Twist& cmd_vel)
{
  linX = cmd_vel.linear.x;
  angZ = cmd_vel.angular.z;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odom_pub");

  ros::NodeHandle n;
  ros::Subscriber cmd_vel = n.subscribe("cmd_vel", 100, chatterCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;



  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linX ;
    cmd_vel.angular.z = angZ ;
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_th = cmd_vel.angular.z * dt;
    double delta_linear_x = cmd_vel.linear.x * dt;
    double delta_angular_th = cmd_vel.angular.z * dt;

    th += delta_th;
    x += delta_linear_x * cos(delta_angular_th);
    y += delta_linear_x * sin(delta_angular_th);
    

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = cmd_vel.linear.x;
    odom.twist.twist.angular.z = cmd_vel.angular.z;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
