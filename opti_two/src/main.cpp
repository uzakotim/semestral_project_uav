// Copyright [2021] [Timur Uzakov]
#include <cv_bridge/cv_bridge.h>


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mrs_msgs/EstimatedState.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <nav_msgs/Odometry.h>

// include opencv2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// using namespace message_filters;

class Node
{
 public:
  Node(char* name_main, char* name_secondary)
  {
    std::string sub_obj_topic           = "";
    std::string sub_obj_topic_secondary = "";
    std::string sub_odom_topic = "";

    sub_obj_topic  += "/";
    sub_obj_topic  += name_main;
    sub_obj_topic  += "/points/";

    sub_obj_topic_secondary  += "/";
    sub_obj_topic_secondary  += name_secondary;
    sub_obj_topic_secondary  += "/points/";
 
    sub_odom_topic  += "/";
    sub_odom_topic  += name_main;
    sub_odom_topic  += "/odometry/odom_main/";

   

    sub_1_.subscribe(nh_, sub_obj_topic, 1);
    sub_2_.subscribe(nh_, sub_obj_topic_secondary, 1);
    sub_3_.subscribe(nh_, sub_odom_topic, 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_,sub_3_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2,_3));
  }

  void callback(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &in1, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &in2, const nav_msgs::OdometryConstPtr &in3)
  {
    ROS_INFO("Synchronization successful");
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_1_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_2_;
  message_filters::Subscriber<nav_msgs::Odometry>                       sub_3_;

  typedef message_filters::sync_policies::ApproximateTime<mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped,nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{ 
  std::string node_name = "";
  node_name += argv[1];
  node_name += "_opti_two";
 
  ros::init(argc, argv, node_name);

  char * first  = argv[1];
  char * second = argv[2];
  Node synchronizer(first,second);

  ros::spin();
}