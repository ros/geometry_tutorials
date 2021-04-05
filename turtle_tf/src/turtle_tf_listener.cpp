#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;

    geometry_msgs::PoseStamped turtle1_pose, turtle1_pose_in_turtle2_frame;
    turtle1_pose.header.frame_id = "turtle1";
    turtle1_pose.pose.orientation.w = 1.0;  // Neutral orientation

    try{
      listener.lookupTransform("/turtle2", "/turtle1",
                               ros::Time(0), transform);
      listener.transformPose("/turtle2", ros::Time(0), turtle1_pose, "world", turtle1_pose_in_turtle2_frame);
      // listener.transformPose("/turtle2", turtle1_pose, turtle1_pose_in_turtle2_frame);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    // ROS_INFO_STREAM("Turtle2 sees turtle1 at x: " << turtle1_pose_in_turtle2_frame.pose.position.x << ", y: " << turtle1_pose_in_turtle2_frame.pose.position.y);

    rate.sleep();
  }
  return 0;
};
