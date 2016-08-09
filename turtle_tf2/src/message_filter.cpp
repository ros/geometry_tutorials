#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PoseDrawer
{
public:
  PoseDrawer() : tf2_(buffer_),  target_frame_("turtle1")
  {
    point_sub_.subscribe(n_, "turtle_point_stamped", 10);
    tf2_filter_ = new tf2_ros::MessageFilter<geometry_msgs::PointStamped>(point_sub_, buffer_, target_frame_, 10, 0);
    tf2_filter_->registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  } ;

private:
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> * tf2_filter_;
  ros::NodeHandle n_;
  std::string target_frame_;

  //  Callback to register with tf2::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr) 
  {
    geometry_msgs::PointStamped point_out;
    try 
    {
      //tf2_.transformPoint(target_frame_, *point_ptr, point_out);
      buffer_.transform(*point_ptr, point_out, target_frame_);
      
      printf("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
             point_out.point.x,
             point_out.point.y,
             point_out.point.z);
    }
    catch (tf2::TransformException &ex) 
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_drawer"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted 
};

