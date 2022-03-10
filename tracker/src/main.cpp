// Copyright [2021] [Timur Uzakov]

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mrs_msgs/EstimatedState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>
#include <cmath>
#define CAMERA_OFFSET 0.2


using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace mrs_msgs;


class Tracker
{

private:
    ros::NodeHandle nh;
    ros::Publisher object_pub;

    message_filters::Subscriber<Odometry> object_sub;
    message_filters::Subscriber<Odometry> pose_sub;
    message_filters::Subscriber<EstimatedState> yaw_sub;
    typedef sync_policies::ApproximateTime<Odometry,Odometry,EstimatedState> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    std::string object_sub_topic = "";
    std::string pose_sub_topic  = "";
    std::string yaw_sub_topic  = "";
    std::string object_pub_topic = "";

    Odometry msg;
    int count {0};
public:

    cv::Mat state, object_coord,object_cov,object_world;
    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    float yaw_value;
    Tracker(char* name)
    {

        object_sub_topic  += "/";
        object_sub_topic  += name;
        object_sub_topic  += "/object/";

        pose_sub_topic += "/";
        pose_sub_topic += name;
        pose_sub_topic +="/odometry/odom_main/";

        yaw_sub_topic += "/";
        yaw_sub_topic += name;
        yaw_sub_topic +="/odometry/heading_state_out";

        object_pub_topic += "/";
        object_pub_topic += name;
        object_pub_topic += "/tracker/";



        object_sub.subscribe(nh,object_sub_topic,1);
        pose_sub.subscribe(nh,pose_sub_topic,1);
        yaw_sub.subscribe(nh,yaw_sub_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), object_sub,pose_sub,yaw_sub));
        sync->registerCallback(boost::bind(&Tracker::callback,this,_1,_2,_3));

        object_pub = nh.advertise<Odometry>(object_pub_topic,1);

        ROS_INFO("All functions initialized");
    }
 

    cv::Mat ObjectCoordinateToWorld(cv::Mat object_position,float yaw_value,cv::Mat drone_position,cv::Mat offset_vector)
    {
        cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << 1280/2,720/2,0); //!TODO! Check image size

        cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) << 0.005,0,0,  0,-0.005,0,     0,0,1); //x same, y flip and rescale



        cv::Mat shifted_and_scaled  = scale_matrix * (object_position - shift_to_center);
        cv::Mat RotationMatrix      = (cv::Mat_<float>(3,3) << 1,0,0, 0,0,1 ,0,1,0); //changing z and y axis
        cv::Mat RotationMatrix2     = (cv::Mat_<float>(3,3) << 0,1,0, 1,0,0, 0,0,1) ; 
        cv::Mat RotationMatrix3     = (cv::Mat_<float>(3,3) << 1,0,0, 0,-1,0, 0,0,1) ;
        cv::Mat RotationMatrix4     = (cv::Mat_<float>(3,3) << cos(yaw_value),-sin(yaw_value),0, sin(yaw_value),cos(yaw_value),0,  0,0,1) ;

        
        cv::Mat not_rot_vector      = RotationMatrix3 * RotationMatrix2 * RotationMatrix * shifted_and_scaled;
        cv::Mat rotated_vector      = RotationMatrix4 * not_rot_vector;
        cv::Mat point = drone_position + rotated_vector + offset_vector;
   
        // ROS_INFO_STREAM("[DRONE]"<<drone_position); 
        // ROS_INFO_STREAM("[NOT ROTATED]"<<not_rot_vector); 
        // ROS_INFO_STREAM("[ROTATED]" << rotated_vector);
        return point;
    }

    void callback(const OdometryConstPtr& object,const OdometryConstPtr& pose, const EstimatedStateConstPtr& yaw)
    {
        ros::Rate rate(100);    
        if (object->pose.pose.position.x != NULL){
            state = (cv::Mat_<float>(3,1)<< pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z);
            object_coord = (cv::Mat_<float>(3,1)<< object->pose.pose.position.x,object->pose.pose.position.y,object->pose.pose.position.z);
            yaw_value = yaw->state[0];
            ROS_INFO_STREAM("[YAW]"<<yaw_value);
            cv::Mat offset = (cv::Mat_<float>(3,1) << (CAMERA_OFFSET*cos(yaw_value)),(CAMERA_OFFSET*sin(yaw_value)),0); // 0.2

            object_world = ObjectCoordinateToWorld(object_coord,yaw_value,state,offset);

            msg.header.frame_id = count;
            msg.header.stamp = ros::Time::now();
            msg.pose.pose.position.x = object_world.at<float>(0);
            msg.pose.pose.position.y = object_world.at<float>(1);
            msg.pose.pose.position.z = object_world.at<float>(2);
            msg.pose.covariance = object->pose.covariance;

            ROS_INFO_STREAM("[GLOBAL]"<< object_world);       
            object_pub.publish(msg);
            rate.sleep();
        }
        else
        {
            yaw_value = yaw->state[0];
            ROS_INFO_STREAM("[YAW]"<<yaw_value);
            ROS_INFO_STREAM("[CANNOT SEE]");
            msg.pose.pose.position.x = NULL;
            msg.pose.pose.position.y = NULL;
            msg.pose.pose.position.z = NULL;
            msg.pose.covariance = msg_cov_array;
            msg.header.frame_id = std::to_string(count);
            msg.header.stamp = ros::Time::now();
            object_pub.publish(msg);
            rate.sleep();
        }
        count++;

    }

};



int main(int argc, char** argv)
{
    ROS_INFO("Tracker node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_tracker";
    ros::init(argc, argv, node_name);
    
    Tracker tr(argv[1]);
    ros::spin();


    return 0;
}
