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

#define RATE 1000

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

int number_of_drones;

cv::Mat convertToCov(const OdometryConstPtr obj)
{
    cv::Mat result = (cv::Mat_<double>(6,6)<<
                                                    obj->pose.covariance[0],
                                                    obj->pose.covariance[1],
                                                    obj->pose.covariance[2], 
                                                    obj->pose.covariance[3],
                                                    obj->pose.covariance[4],
                                                    obj->pose.covariance[5],
                                                    obj->pose.covariance[6],
                                                    obj->pose.covariance[7],
                                                    obj->pose.covariance[8],
                                                    obj->pose.covariance[9],
                                                    obj->pose.covariance[10],
                                                    obj->pose.covariance[11],
                                                    obj->pose.covariance[12], 
                                                    obj->pose.covariance[13],
                                                    obj->pose.covariance[14],
                                                    obj->pose.covariance[15],
                                                    obj->pose.covariance[16],
                                                    obj->pose.covariance[17],
                                                    obj->pose.covariance[18],
                                                    obj->pose.covariance[19],
                                                    obj->pose.covariance[20],
                                                    obj->pose.covariance[21],
                                                    obj->pose.covariance[22], 
                                                    obj->pose.covariance[23],
                                                    obj->pose.covariance[24],
                                                    obj->pose.covariance[25],
                                                    obj->pose.covariance[26],
                                                    obj->pose.covariance[27],
                                                    obj->pose.covariance[28],
                                                    obj->pose.covariance[29],
                                                    obj->pose.covariance[30],
                                                    obj->pose.covariance[31],
                                                    obj->pose.covariance[32], 
                                                    obj->pose.covariance[33],
                                                    obj->pose.covariance[34],
                                                    obj->pose.covariance[35]
                                                    );
    return result;
}
class SensFuseThree
{

public:

    ros::Publisher sf1_pub;
    ros::Publisher goal_pub;
    nav_msgs::Odometry goal_msg;

    message_filters::Subscriber<Odometry> obj_sub;
    message_filters::Subscriber<Odometry> obj_secondary_sub;
    message_filters::Subscriber<Odometry> obj_third_sub;
    message_filters::Subscriber<Odometry> pose_sub;

    std::string obj_topic           = "";
    std::string obj_topic_secondary = "";
    std::string obj_topic_third     = "";
    std::string pose_topic          = "";
    std::string goal_topic          = "";

    //---Kalman Filter Parameters---->>----

    cv::KalmanFilter KF = cv::KalmanFilter(6,6,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(6,1);

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    cv::Point3f center3D;

    cv::Mat main_cov,secondary_cov, third_cov;
    // ---<< Kalman Filter Parameters ----
    typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;




    SensFuseThree(ros::NodeHandle nh,char* name_main, char* name_secondary, char* name_third)
    {
        
        obj_topic  += "/";
        obj_topic  += name_main;
        obj_topic  += "/tracker/";

        obj_topic_secondary  += "/";
        obj_topic_secondary  += name_secondary;
        obj_topic_secondary  += "/tracker/";

        // if three drones
        obj_topic_third  += "/";
        obj_topic_third  += name_third;
        obj_topic_third  += "/tracker/";

        pose_topic += "/";
        pose_topic += name_main;
        pose_topic += "/odometry/odom_main/";

        goal_topic += "/";
        goal_topic += name_main;
        goal_topic += "/goal/";

        obj_sub.subscribe(nh,obj_topic,1);
        obj_secondary_sub.subscribe (nh,obj_topic_secondary,1);
        obj_third_sub.subscribe (nh,obj_topic_third,1);
        pose_sub.subscribe(nh,pose_topic,1);

        goal_pub = nh.advertise<Odometry>(goal_topic,1);        
    

        sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub,obj_third_sub,pose_sub));
        sync->registerCallback(boost::bind(&SensFuseThree::callback_three,this, _1,_2,_3,_4));


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
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1e-4)); //Q
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));   //R
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----


        ROS_INFO("[Three Drones] All functions initialized");
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

    void callback_three(const OdometryConstPtr obj,const OdometryConstPtr obj_secondary, const OdometryConstPtr obj_third, const OdometryConstPtr pose)
    {
        ros::Rate rate(RATE);
        ROS_INFO_STREAM("Synchronized");
        // if I see...
        if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x=='\0')) && (obj_third->pose.pose.position.x=='\0'))
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            center3D.x = (float)(obj->pose.pose.position.x);
            center3D.y = (float)(obj->pose.pose.position.y);
            center3D.z = (float)(obj->pose.pose.position.z);

            main_cov = convertToCov(obj);    
            main_cov.convertTo(main_cov, CV_32F);

            KF.measurementNoiseCov = main_cov;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            // ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        }
        // if second sees...
        else if (((obj->pose.pose.position.x=='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x=='\0'))
        
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            center3D.x = (float)(obj_secondary->pose.pose.position.x);
            center3D.y = (float)(obj_secondary->pose.pose.position.y);
            center3D.z = (float)(obj_secondary->pose.pose.position.z);

            secondary_cov = convertToCov(obj_secondary);

            secondary_cov.convertTo(secondary_cov, CV_32F);

            KF.measurementNoiseCov = secondary_cov;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            // ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        }   
         // if third sees...
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x=='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            center3D.x = (float)(obj_third->pose.pose.position.x);
            center3D.y = (float)(obj_third->pose.pose.position.y);
            center3D.z = (float)(obj_third->pose.pose.position.z);

            third_cov = convertToCov(obj_third);
            third_cov.convertTo(third_cov, CV_32F);

            KF.measurementNoiseCov = third_cov;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            //ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        }  
        // if 1 & 2 sees
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x=='\0'))
        
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            
            center3D.x = (float)((obj->pose.pose.position.x + obj_secondary->pose.pose.position.x)/2.0);
            center3D.y = (float)((obj->pose.pose.position.y + obj_secondary->pose.pose.position.y)/2.0);
            center3D.z = (float)((obj->pose.pose.position.z + obj_secondary->pose.pose.position.z)/2.0);

            main_cov = convertToCov(obj);
            secondary_cov = convertToCov(obj_secondary);

            main_cov.convertTo(main_cov, CV_32F);
            secondary_cov.convertTo(secondary_cov, CV_32F);

            KF.measurementNoiseCov = (main_cov+secondary_cov)/2.0;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            //ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        } 
        // if 1 & 3 sees
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x=='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            
            center3D.x = (float)((obj->pose.pose.position.x + obj_third->pose.pose.position.x)/2.0);
            center3D.y = (float)((obj->pose.pose.position.y + obj_third->pose.pose.position.y)/2.0);
            center3D.z = (float)((obj->pose.pose.position.z + obj_third->pose.pose.position.z)/2.0);

            main_cov = convertToCov(obj);
            third_cov = convertToCov(obj_third);

            main_cov.convertTo(main_cov, CV_32F);
            third_cov.convertTo(third_cov, CV_32F);

            KF.measurementNoiseCov = (main_cov+third_cov)/2.0;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            //ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        }    
         // if 2 & 3 sees
        else if (((obj->pose.pose.position.x=='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            
            center3D.x = (float)((obj_secondary->pose.pose.position.x + obj_third->pose.pose.position.x)/2.0);
            center3D.y = (float)((obj_secondary->pose.pose.position.y + obj_third->pose.pose.position.y)/2.0);
            center3D.z = (float)((obj_secondary->pose.pose.position.z + obj_third->pose.pose.position.z)/2.0);

            secondary_cov = convertToCov(obj_secondary);
            third_cov = convertToCov(obj_third);
            secondary_cov.convertTo(secondary_cov, CV_32F);
            third_cov.convertTo(third_cov, CV_32F);

            KF.measurementNoiseCov = (secondary_cov+third_cov)/2.0;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            //ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
            goal_msg.pose.covariance = msg_cov_array;

            goal_msg.header.stamp = ros::Time::now();    
            goal_pub.publish(goal_msg);
            rate.sleep();
        }  
         // if all 3 see...
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            cv::Point3f predictPt = PredictUsingKalmanFilter();
            
            center3D.x = (float)((obj->pose.pose.position.x + obj_secondary->pose.pose.position.x + obj_third->pose.pose.position.x)/3.0);
            center3D.y = (float)((obj->pose.pose.position.y + obj_secondary->pose.pose.position.y + obj_third->pose.pose.position.y)/3.0);
            center3D.z = (float)((obj->pose.pose.position.z + obj_secondary->pose.pose.position.z + obj_third->pose.pose.position.z)/3.0);

            main_cov = convertToCov(obj);
            secondary_cov = convertToCov(obj_secondary);
            third_cov = convertToCov(obj_third);
            
            main_cov.convertTo(main_cov, CV_32F);
            secondary_cov.convertTo(secondary_cov, CV_32F);
            third_cov.convertTo(third_cov, CV_32F);

            KF.measurementNoiseCov = (main_cov + secondary_cov+third_cov)/3.0;

            SetMeasurement(center3D);

            cv::Point3f statePt = UpdateKalmanFilter(measurement);

            goal_msg.pose.pose.position.x = (double)statePt.x;
            goal_msg.pose.pose.position.y = (double)statePt.y;
            goal_msg.pose.pose.position.z = (double)statePt.z;

            //ROS_INFO_STREAM(statePt);
            cov_matrix = KF.errorCovPost;
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
    SensFuseThree sf(nh,argv[1],argv[2],argv[3]);

    ros::spin();

    return 0;
}
