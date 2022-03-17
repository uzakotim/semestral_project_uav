// Copyright [2021] [Timur Uzakov]

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Odometry.h>
#include <cmath>
#include <ros/ros.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <string.h>


using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class SensFuse
{
public:


    ros::Publisher sf1_pub;
    ros::Publisher goal_pub;
    nav_msgs::Odometry goal_msg;

    message_filters::Subscriber<Odometry> obj_sub;
    message_filters::Subscriber<Odometry> obj_secondary_sub;
    message_filters::Subscriber<Odometry> pose_sub;

    std::string obj_topic           = "";
    std::string obj_topic_secondary = "";
    std::string pose_topic          = "";
    std::string goal_topic          = "";


    typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


    //---Kalman Filter Parameters---->>----

    cv::KalmanFilter KF = cv::KalmanFilter(6,6,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(6,1);

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    cv::Point3f center3D;

    // ---<< Kalman Filter Parameters ----




    SensFuse(ros::NodeHandle nh,char* name_main, char* name_secondary)
    {
        obj_topic  += "/";
        obj_topic  += name_main;
        obj_topic  += "/tracker/";

        obj_topic_secondary  += "/";
        obj_topic_secondary  += name_secondary;
        obj_topic_secondary  += "/tracker/";

        pose_topic += "/";
        pose_topic += name_main;
        pose_topic += "/odometry/odom_main/";

        goal_topic += "/";
        goal_topic += name_main;
        goal_topic += "/goal/";

        obj_sub.subscribe (nh,obj_topic,1);
        obj_secondary_sub.subscribe (nh,obj_topic_secondary,1);
        pose_sub.subscribe  (nh,pose_topic,1);

        goal_pub = nh.advertise<Odometry>(goal_topic,1);


        sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub,pose_sub));
        sync->registerCallback(boost::bind(&SensFuse::callback,this, _1,_2,_3));


        //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(6,6) <<  1,0,0,1,0,0,
                                                        0,1,0,0,1,0,
                                                        0,0,1,0,0,1,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        measurement.setTo(cv::Scalar(0));
        KF.statePre.at<float>(0) = 0;
        KF.statePre.at<float>(1) = 0;
        KF.statePre.at<float>(2) = 0;
        KF.statePre.at<float>(3) = 0;
        KF.statePre.at<float>(4) = 0;
        KF.statePre.at<float>(5) = 0;

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1e-4));
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----


        ROS_INFO("All functions initialized");
    }
    cv::Point3f PredictUsingKalmanFilter()
    {
        // Prediction, to update the internal statePre variable -->>
        cv::Mat prediction  =  KF.predict();
        cv::Point3f predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
        return  predictPt;
        //  <<---Prediction, to update the internal statePre variable
    }

    void SetMeasurement(cv::Point3f center)
    {
        measurement.at<float>(0) = center.x;
        measurement.at<float>(1) = center.y;
        measurement.at<float>(2) = center.z;
    }

    cv::Point3f UpdateKalmanFilter(cv::Mat_<float>  measurement)
    {
        // Updating Kalman Filter
        cv::Mat       estimated = KF.correct(measurement);
        cv::Point3f   statePt(estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
        cv::Point3f   measPt(measurement(0),measurement(1),measurement(2));
        return      statePt;
    }

    void callback(const OdometryConstPtr obj,const OdometryConstPtr obj_secondary, const OdometryConstPtr pose)
    {
        ros::Rate rate(100);
        ROS_INFO_STREAM("Synchronized");

        if ((obj->pose.pose.position.x!='\0') || (obj_secondary->pose.pose.position.x!='\0'))
        {

            cv::Point3f predictPt = PredictUsingKalmanFilter();
            
            center3D.x = (float)((obj->pose.pose.position.x + obj_secondary->pose.pose.position.x)/2.0);
            center3D.y = (float)((obj->pose.pose.position.y + obj_secondary->pose.pose.position.y)/2.0);
            center3D.z = (float)((obj->pose.pose.position.z + obj_secondary->pose.pose.position.z)/2.0);

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        }
        else
        {
            goal_msg.pose.pose.position.x = '\0';
            goal_msg.pose.pose.position.y = '\0';
            goal_msg.pose.pose.position.z = '\0';
            goal_msg.pose.covariance = msg_cov_array;
            goal_msg.header.stamp = ros::Time::now();
            goal_pub.publish(goal_msg);
   
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ROS_INFO("Sensor Fusion node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_sensor_fusion";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    SensFuse sf(nh,argv[1],argv[2]);
    ros::spin();

    return 0;
}
