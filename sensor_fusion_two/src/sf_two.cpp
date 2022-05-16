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
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/EstimatedState.h>

// include ros library
#include <ros/ros.h>
#include "ros/service_client.h"

#define RATE 1000

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace mrs_msgs;

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
class SensFuseTwo
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

    std::string pub_pose_topic      = "";


    //---Kalman Filter Parameters---->>----

    cv::KalmanFilter KF = cv::KalmanFilter(6,6,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(6,1);

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    cv::Point3f center3D;

    cv::Mat main_cov,secondary_cov;
    // ---<< Kalman Filter Parameters ----

    // ---------OUTPUT MSG-----------------------------------
    boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
    ros::ServiceClient client;
    mrs_msgs::Vec4 srv;

    // ----------------------input --------------------------
    
    float                      offset_x;
    float                      offset_y;
    float                      offset_z;

    // ------------------------------------------------------
    typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


   


    SensFuseTwo(ros::NodeHandle nh,char* name_main, char* name_secondary,float x_parameter,float y_parameter, float z_parameter)
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

        pub_pose_topic+="/";
        pub_pose_topic+=name_main;
        pub_pose_topic+="/control_manager/goto/";


        obj_sub.subscribe(nh,obj_topic,1);
        obj_secondary_sub.subscribe (nh,obj_topic_secondary,1);
        pose_sub.subscribe(nh,pose_topic,1);

        goal_pub = nh.advertise<Odometry>(goal_topic,1);


        sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub,pose_sub));
        sync->registerCallback(boost::bind(&SensFuseTwo::callback_two,this, _1,_2,_3));

        client = nh.serviceClient<mrs_msgs::Vec4>(pub_pose_topic);

        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;


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
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1e-1)); //Q //-4
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));   //R
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----


        ROS_INFO("[Two Drones] All functions initialized");
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
    void callback_two(const OdometryConstPtr obj,const OdometryConstPtr obj_secondary, const OdometryConstPtr pose)
    {
        ros::Rate rate(RATE);
        ROS_INFO_STREAM("Synchronized");
        // if we see...
        if ( (obj->pose.pose.position.x!='\0') &&  (obj_secondary->pose.pose.position.x!='\0') )
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

            // goal_msg.pose.pose.position.x = (double)statePt.x;
            // goal_msg.pose.pose.position.y = (double)statePt.y;
            // goal_msg.pose.pose.position.z = (double)statePt.z;
            
            // cov_matrix = KF.errorCovPost;
            
            // goal_msg.pose.covariance = msg_cov_array;
            // goal_msg.header.stamp = ros::Time::now();    
            // goal_pub.publish(goal_msg);

            // NEW PART --------------------------------------------
            float x_goal = (float)statePt.x;
            float y_goal = (float)statePt.y;
            float z_goal = (float)statePt.z;

            ROS_INFO_STREAM("[Destination]: "<<x_goal<<" : "<<y_goal<<" : "<<z_goal);

            //------------MEASUREMENTS------------------------------
            cv::Mat state = (cv::Mat_<float>(3,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z);
            cv::Mat state_cov = SensFuseTwo::convertToCov(pose);
            //------------------------------------------------------

            // MRS - waypoint --------------------------------------
            srv.request.goal = boost::array<float, 4>{x_goal+offset_x,y_goal+offset_y,z_goal+offset_z,std::round(atan2(y_goal-state.at<float>(1),x_goal-state.at<float>(0)))};

            if (client.call(srv))
            {
                ROS_INFO("Successfull calling service\n");
            }
            else 
            {
                ROS_ERROR("Could not publish\n");
            }

            // -----------------------------------------------------


            rate.sleep();
        }
        // if I see...
        else if (obj->pose.pose.position.x!='\0')
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

            // NEW PART --------------------------------------------
            float x_goal = (float)statePt.x;
            float y_goal = (float)statePt.y;
            float z_goal = (float)statePt.z;

            ROS_INFO_STREAM("[Destination]: "<<x_goal<<" : "<<y_goal<<" : "<<z_goal);

            //------------MEASUREMENTS------------------------------
            cv::Mat state = (cv::Mat_<float>(3,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z);
            cv::Mat state_cov = SensFuseTwo::convertToCov(pose);
            //------------------------------------------------------

            // MRS - waypoint --------------------------------------
            srv.request.goal = boost::array<float, 4>{x_goal+offset_x,y_goal+offset_y,z_goal+offset_z,std::round(atan2(y_goal-state.at<float>(1),x_goal-state.at<float>(0)))};

            if (client.call(srv))
            {
                ROS_INFO("Successfull calling service\n");
            }
            else 
            {
                ROS_ERROR("Could not publish\n");
            }

            // -----------------------------------------------------

            // goal_msg.pose.pose.position.x = (double)statePt.x;
            // goal_msg.pose.pose.position.y = (double)statePt.y;
            // goal_msg.pose.pose.position.z = (double)statePt.z;

            // ROS_INFO_STREAM(statePt);
            // cov_matrix = KF.errorCovPost;
            // goal_msg.pose.covariance = msg_cov_array;

            // goal_msg.header.stamp = ros::Time::now();    
            // goal_pub.publish(goal_msg);

            rate.sleep();
        }
        // if another sees...
        else if (obj_secondary->pose.pose.position.x!='\0')
        
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
            // NEW PART --------------------------------------------
            float x_goal = (float)statePt.x;
            float y_goal = (float)statePt.y;
            float z_goal = (float)statePt.z;

            ROS_INFO_STREAM("[Destination]: "<<x_goal<<" : "<<y_goal<<" : "<<z_goal);

            //------------MEASUREMENTS------------------------------
            cv::Mat state = (cv::Mat_<float>(3,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z);
            cv::Mat state_cov = SensFuseTwo::convertToCov(pose);
            //------------------------------------------------------

            // MRS - waypoint --------------------------------------
            srv.request.goal = boost::array<float, 4>{x_goal+offset_x,y_goal+offset_y,z_goal+offset_z,std::round(atan2(y_goal-state.at<float>(1),x_goal-state.at<float>(0)))};

            if (client.call(srv))
            {
                ROS_INFO("Successfull calling service\n");
            }
            else 
            {
                ROS_ERROR("Could not publish\n");
            }

            // -----------------------------------------------------


           // goal_msg.pose.pose.position.x = (double)statePt.x;
            // goal_msg.pose.pose.position.y = (double)statePt.y;
            // goal_msg.pose.pose.position.z = (double)statePt.z;

            // ROS_INFO_STREAM(statePt);
            // cov_matrix = KF.errorCovPost;
            // goal_msg.pose.covariance = msg_cov_array;

            // goal_msg.header.stamp = ros::Time::now();    
            // goal_pub.publish(goal_msg);
            rate.sleep();
        }   
    }
};




int main(int argc, char** argv)
{
    ROS_INFO("Sensor Fusion node initialized");
 
    if(argc < 4) 
    {
        std::cerr<<"Please, enter the drone position around the goal, in the following form: UAV_NAME_OWN UAV_NAME_SEC x y z "<<'\n';
        return 1; 
    }

    std::string node_name = "";
    node_name += argv[1];
    node_name += "_sensor_fusion";
  
    std::istringstream              source_cmd_x(argv[3]);
    std::istringstream              source_cmd_y(argv[4]);
    std::istringstream              source_cmd_z(argv[5]);
    
    float offset_parameter_x;
    float offset_parameter_y;
    float offset_parameter_z;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;
 

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    SensFuseTwo sf(nh,argv[1],argv[2],offset_parameter_x,offset_parameter_y,offset_parameter_z);
    ros::spin();

    return 0;
}
