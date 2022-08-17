// Copyright [2021] [Timur Uzakov]

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Odometry.h>
#include <cmath>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <string.h>


#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/EstimatedState.h>

// include ros library
#include <ros/ros.h>
#include "ros/service_client.h"

#define CALCULATION_STEPS 10 //150
#define CALCULATIOM_STEPS_IN_MOTION 5 //30

#define CONTROLLER_PERIOD 0.1 
// the most optimal value is 1, the less the faster motion

#define CONTROL_GAIN_GOAL 200 //20

#define CONTROL_GAIN_DISTANCE 0.0 
#define CONTROL_DISTANCE 2.0 

#define CONTROL_GAIN_STATE_Z 0.01 // 100
#define CONTROL_GAIN_STATE 0.1  // 1
// influences how sharp are drone's motions - the lower the sharper

#define DELTA_MAX 0.5 //0.5
// determines how fast drone optimises - the smaller the faster
#define RADIUS 0.0
#define SEARCH_SIZE 8
#define SEARCH_HEIGHT 3.0




using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace mrs_msgs;


int number_of_drones;


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
    message_filters::Subscriber<EstimatedState> yaw_sub;


    std::string sub_obj_topic           = "";
    std::string sub_obj_topic_secondary = "";
    std::string sub_obj_topic_third     = "";
    std::string sub_pose_topic          = "";
    std::string sub_yaw_topic           = "";

    std::string pub_pose_topic          = "";
    std::string pub_goal_topic          = "";

    //---Kalman Filter Parameters---->>----

    cv::KalmanFilter KF = cv::KalmanFilter(6,6,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(6,1);

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    cv::Point3f center3D;

    cv::Mat main_cov,secondary_cov, third_cov;
    // ---<< Kalman Filter Parameters ----

    // ---------OUTPUT MSG-----------------------------------
    boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
    ros::ServiceClient client;
    mrs_msgs::ReferenceStampedSrv srv;

    //---------------------------------------------------------
     //------------OPTIMIZATION-PARAMETERS----------------`
    float                      center_x {0.0};
    float                      center_y {0.0};
    float                      center_z {0.0};
    
    float                      x_previous;
    float                      y_previous;
    float                      z_previous;

    float                      init_offset_x;
    float                      init_offset_y;
    float                      init_offset_z;

    float                      offset_x;
    float                      offset_y;
    float                      offset_z;

    // mode
    int                         around;

    float angle = 0.0;
    float searchAngle = -M_PI;


    float radius = RADIUS;
    
    float goal_x {0.0};
    float goal_y {0.0};
    float goal_z {0.0};
    float goal_yaw{0.0};
    
    const float heights[6] {2.5,2.7,3.0,3.2,3.0,2.7};

    float pose_x {0.0};
    float pose_y {0.0};

    float init_x {0.0};
    float init_y {0.0};
    float init_z {0.0};

    float obj_x {0.0};
    float obj_y {0.0};
    float obj_z {0.0};
    float obj_yaw {0.0};
    
    float error {0.0};
    float yaw_value {0.0};
    float stored_yaw{0.0};

    // ----------Formation controller parameters--------------

    // parameters
    float n_pos {1.2};
    float n_neg {0.5};
    float delta_max {DELTA_MAX};
    float delta_min {0.001}; // 0.000001
   
    float cost_prev_x{0},cost_cur_x{0};
    float cost_prev_y{0},cost_cur_y{0};
    float cost_prev_z{0},cost_cur_z{0};
    float cost_prev_yaw{0},cost_cur_yaw{0};

    float cost_dif_x{0};
    float cost_dif_y{0};
    float cost_dif_z{0};
    float cost_dif_yaw{0};

    float step_x{0};
    float step_y{0};
    float step_z{0};
    float step_yaw{0};

    std::vector<float> cost_cur;
    std::vector<float> cost_prev;
    std::vector<float> grad_cur {0,0,0,0}; // 0 0 0 
    std::vector<float> grad_prev {0,0,0,0}; // 0 0 0

    std::vector<float> delta {0.5,0.5,0.5,0.5};
    std::vector<float> delta_prev {0.5,0.5,0.5,0.5};

    int k{0};  //computing steps
    int count {0};
    int height_count {0};
    
    cv::Mat w_prev = (cv::Mat_<float>(4,1) <<  0,0,0,0);
    cv::Mat w;
    cv::Mat master_pose;
    
    cv::Mat state,state_cov,obj_cov;

    cv::Mat tracker_vector = (cv::Mat_<float>(4,1) << 0,0,0,0);
    
    std::vector<cv::Mat> tracker;

    float resulting_cost_x {0};
    float resulting_cost_y {0};
    float resulting_cost_z {0};
    float resulting_cost_yaw {0};

    //-----------------------------------------------------------------------------------------------------------------
    typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry,Odometry,EstimatedState> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


    SensFuseThree(ros::NodeHandle nh,char* name_main, char* name_secondary, char* name_third,float x_parameter,float y_parameter, float z_parameter)
    {
        //subscribers

        sub_obj_topic  += "/";
        sub_obj_topic  += name_main;
        sub_obj_topic  += "/tracker/";

        sub_obj_topic_secondary  += "/";
        sub_obj_topic_secondary  += name_secondary;
        sub_obj_topic_secondary  += "/tracker/";

        // if three drones
        sub_obj_topic_third  += "/";
        sub_obj_topic_third  += name_third;
        sub_obj_topic_third  += "/tracker/";

        sub_yaw_topic  += "/";
        sub_yaw_topic  += name_main;
        sub_yaw_topic  += "/odometry/heading_state_out/";

        sub_pose_topic += "/";
        sub_pose_topic += name_main;
        sub_pose_topic += "/odometry/odom_main/";


        // publishers
        pub_goal_topic += "/";
        pub_goal_topic += name_main;
        pub_goal_topic += "/goal/";

        pub_pose_topic += "/";
        pub_pose_topic += name_main;
        pub_pose_topic += "/control_manager/reference";

        obj_sub.subscribe(nh,sub_obj_topic,1);
        obj_secondary_sub.subscribe (nh,sub_obj_topic_secondary,1);
        obj_third_sub.subscribe (nh,sub_obj_topic_third,1);
        pose_sub.subscribe(nh,sub_pose_topic,1);
        yaw_sub.subscribe(nh,sub_yaw_topic,1);

        goal_pub = nh.advertise<Odometry>(pub_goal_topic,1);        
    

        sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub,obj_third_sub,pose_sub,yaw_sub));
        sync->registerCallback(boost::bind(&SensFuseThree::callback_three,this, _1,_2,_3,_4,_5));

        client = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>(pub_pose_topic);
        
        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;
        
        init_offset_x = x_parameter;
        init_offset_y = y_parameter;
        init_offset_z = z_parameter;


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
        obj_cov             = (cv::Mat_<float>(6,6) <<  1,0,0,0,0,0,
                                                        0,1,0,0,0,0,
                                                        0,0,1,0,0,0,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);


        ROS_INFO("[Three Drones] All functions initialized");
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
    cv::Point3f updateKFSingle(const OdometryConstPtr obj)
    {
        cv::Point3f predictPt = PredictUsingKalmanFilter();
        center3D.x = (float)(obj->pose.pose.position.x);
        center3D.y = (float)(obj->pose.pose.position.y);
        center3D.z = (float)(obj->pose.pose.position.z);

        main_cov = SensFuseThree::convertToCov(obj);    
        main_cov.convertTo(main_cov, CV_32F);

        KF.measurementNoiseCov = main_cov;
        obj_cov = main_cov;
        SetMeasurement(center3D);

        cv::Point3f statePt = UpdateKalmanFilter(measurement);
        return statePt;
    }
    cv::Point3f updateKFDouble(const OdometryConstPtr obj,const OdometryConstPtr obj_secondary)
    {
         cv::Point3f predictPt = PredictUsingKalmanFilter();
            
        center3D.x = (float)((obj->pose.pose.position.x + obj_secondary->pose.pose.position.x)/2.0);
        center3D.y = (float)((obj->pose.pose.position.y + obj_secondary->pose.pose.position.y)/2.0);
        center3D.z = (float)((obj->pose.pose.position.z + obj_secondary->pose.pose.position.z)/2.0);

        main_cov = SensFuseThree::convertToCov(obj);
        secondary_cov = convertToCov(obj_secondary);

        main_cov.convertTo(main_cov, CV_32F);
        secondary_cov.convertTo(secondary_cov, CV_32F);

        KF.measurementNoiseCov = (main_cov+secondary_cov)/2.0;
        obj_cov = (main_cov+secondary_cov)/2.0;
        SetMeasurement(center3D);

        cv::Point3f statePt = UpdateKalmanFilter(measurement);

        return statePt;
    }

    cv::Point3f updateKFTriple(const OdometryConstPtr obj,const OdometryConstPtr obj_secondary,const OdometryConstPtr obj_third)
    {
        cv::Point3f predictPt = PredictUsingKalmanFilter();
            
        center3D.x = (float)((obj->pose.pose.position.x + obj_secondary->pose.pose.position.x + obj_third->pose.pose.position.x)/3.0);
        center3D.y = (float)((obj->pose.pose.position.y + obj_secondary->pose.pose.position.y + obj_third->pose.pose.position.y)/3.0);
        center3D.z = (float)((obj->pose.pose.position.z + obj_secondary->pose.pose.position.z + obj_third->pose.pose.position.z)/3.0);

        main_cov = SensFuseThree::convertToCov(obj);
        secondary_cov = convertToCov(obj_secondary);
        third_cov = convertToCov(obj_third);
        
        main_cov.convertTo(main_cov, CV_32F);
        secondary_cov.convertTo(secondary_cov, CV_32F);
        third_cov.convertTo(third_cov, CV_32F);
        obj_cov = (main_cov+third_cov)/3.0;

        KF.measurementNoiseCov = (main_cov + secondary_cov+third_cov)/3.0;

        SetMeasurement(center3D);

        cv::Point3f statePt = UpdateKalmanFilter(measurement);

        return statePt;
    }
    //--------  COST FUNCTIONS ------------------------------
    float CostX(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,float offset_x)
    {        
        // resulting_cost_x = CONTROL_GAIN_STATE*std::pow((w.at<float>(0) - w_prev.at<float>(0)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(0) - (master_pose.at<float>(0)+offset_x)),2)+ 0.5*cv::determinant(state_cov)*10e-1; //-10 -4  
        resulting_cost_x = CONTROL_GAIN_STATE*std::pow((w.at<float>(0) - w_prev.at<float>(0)),2) + CONTROL_GAIN_DISTANCE*(CONTROL_DISTANCE - std::pow((w.at<float>(0) - (master_pose.at<float>(0))),2)) +CONTROL_GAIN_GOAL*std::pow((w.at<float>(0) - (master_pose.at<float>(0)+offset_x)),2) + cv::determinant(state_cov)*10e-1 + cv::determinant(object_cov)*10e-1; //-10 -4  
        return resulting_cost_x;
    }
    float CostY(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,float offset_y)
    {
        // resulting_cost_y = CONTROL_GAIN_STATE*std::pow((w.at<float>(1) - w_prev.at<float>(1)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(1) - (master_pose.at<float>(1)+offset_y)),2)+ 0.5*cv::determinant(state_cov)*10e-1;  
        resulting_cost_y = CONTROL_GAIN_STATE*std::pow((w.at<float>(1) - w_prev.at<float>(1)),2)  + CONTROL_GAIN_DISTANCE*(CONTROL_DISTANCE - std::pow((w.at<float>(1) - (master_pose.at<float>(1))),2))+ CONTROL_GAIN_GOAL*std::pow((w.at<float>(1) - (master_pose.at<float>(1)+offset_y)),2) + cv::determinant(state_cov)*10e-1 + cv::determinant(object_cov)*10e-1;  
        return resulting_cost_y;
    }
    float CostZ(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,float offset_z)
    {
        // resulting_cost_z = CONTROL_GAIN_STATE*std::pow((w.at<float>(2) - w_prev.at<float>(2)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2)+ 0.5*cv::determinant(state_cov)*10e-1;  
        resulting_cost_z = CONTROL_GAIN_STATE_Z*std::pow((w.at<float>(2) - w_prev.at<float>(2)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2) + cv::determinant(state_cov)*10e-1 + cv::determinant(object_cov)*10e-1;  
        return resulting_cost_z;
    }

    float CostYaw(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov)
    {
        // resulting_cost_yaw = CONTROL_GAIN_STATE*std::pow((w.at<float>(3) - w_prev.at<float>(2)),3) +  CONTROL_GAIN_GOAL*std::pow((w.at<float>(3) - (master_pose.at<float>(3))),2);  
        resulting_cost_yaw = CONTROL_GAIN_STATE*std::pow((w.at<float>(3) - w_prev.at<float>(3)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(3) - (master_pose.at<float>(3))),2) + cv::determinant(state_cov)*10e-1  + cv::determinant(object_cov)*10e-1;  
        return resulting_cost_yaw;
    }
    int sign(float x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

    
    cv::Mat calculateFormation(cv::Mat w, cv::Mat master_pose,cv::Mat state_cov,cv::Mat object_cov)
    {
    //------------------------------------------------------
        //------------RPROP----------------
        // goal-driven behaviour
        cost_prev_x = CostX(w_prev,w_prev,master_pose,state_cov,object_cov,offset_x);
        cost_prev_y = CostY(w_prev,w_prev,master_pose,state_cov,object_cov,offset_y);
        cost_prev_z = CostZ(w_prev,w_prev,master_pose,state_cov,object_cov,offset_z);
        cost_prev_yaw = CostYaw(w_prev,w_prev,master_pose,state_cov,object_cov);

        cost_cur_x = CostX(w,w_prev,master_pose,state_cov,object_cov,offset_x);
        cost_cur_y = CostY(w,w_prev,master_pose,state_cov,object_cov,offset_y);
        cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,object_cov,offset_z);
        cost_cur_yaw = CostYaw(w,w_prev,master_pose,state_cov,object_cov);

        cost_cur.push_back(cost_cur_x);
        cost_cur.push_back(cost_cur_y);
        cost_cur.push_back(cost_cur_z);
        cost_cur.push_back(cost_cur_yaw);
        
        cost_prev.push_back(cost_prev_x);
        cost_prev.push_back(cost_prev_y);
        cost_prev.push_back(cost_prev_z);
        cost_prev.push_back(cost_prev_yaw);

        cost_dif_x = (cost_cur_x - cost_prev_x);
        cost_dif_y = (cost_cur_y - cost_prev_y);
        cost_dif_z = (cost_cur_z - cost_prev_z);
        cost_dif_yaw = (cost_cur_yaw - cost_prev_yaw);


        step_x = w.at<float>(0) - w_prev.at<float>(0);
        step_y = w.at<float>(1) - w_prev.at<float>(1);
        step_z = w.at<float>(2) - w_prev.at<float>(2);
        step_yaw = w.at<float>(3) - w_prev.at<float>(3);

        grad_prev.push_back(cost_dif_x/step_x);
        grad_prev.push_back(cost_dif_y/step_y);
        grad_prev.push_back(cost_dif_z/step_z);
        grad_prev.push_back(cost_dif_yaw/step_yaw);

        // computing longer when standing
        if ((std::abs(w.at<float>(0) - w_prev.at<float>(0))<0.2) || (std::abs(w.at<float>(1) - w_prev.at<float>(1))<0.2) || (std::abs(w.at<float>(2) - w_prev.at<float>(2))<0.2))
        {
                k = CALCULATION_STEPS;
        } else  k = CALCULATIOM_STEPS_IN_MOTION;
        // -------------------------------------------------- 
    
        for(int j=0;j<k;j++)
        {
            // Main RPROP loop
            cost_cur_x = CostX(w,w_prev,master_pose,state_cov,object_cov,offset_x);
            cost_cur_y = CostY(w,w_prev,master_pose,state_cov,object_cov,offset_y);
            cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,object_cov,offset_z);
            cost_cur_yaw = CostYaw(w,w_prev,master_pose,state_cov,object_cov);

            cost_cur[0] = cost_cur_x;
            cost_cur[1] = cost_cur_y;
            cost_cur[2] = cost_cur_z;
            cost_cur[3] = cost_cur_yaw;

            cost_dif_x = (cost_cur_x - cost_prev_x);
            cost_dif_y = (cost_cur_y - cost_prev_y);
            cost_dif_z = (cost_cur_z - cost_prev_z);
            cost_dif_yaw = (cost_cur_yaw - cost_prev_yaw);

            step_x = w.at<float>(0) - w_prev.at<float>(0);
            step_y = w.at<float>(1) - w_prev.at<float>(1);
            step_z = w.at<float>(2) - w_prev.at<float>(2);
            step_yaw = w.at<float>(3) - w_prev.at<float>(3);
            
            grad_cur[0] = cost_dif_x/step_x;
            grad_cur[1] = cost_dif_y/step_y;
            grad_cur[2] = cost_dif_z/step_z;
            grad_cur[3] = cost_dif_yaw/step_yaw;

            delta_prev = delta; 
            for (int i = 0; i<4;i++)
            {
                if ((grad_prev[i]*grad_cur[i])>0)
                {
                    delta[i] = std::min(delta_prev[i]*n_pos,delta_max);
                    w_prev.at<float>(i) = w.at<float>(i);
                    w.at<float>(i) = w.at<float>(i) - sign(grad_cur[i])*delta[i];
                    grad_prev[i] = grad_cur[i]; 
                } else if ((grad_prev[i]*grad_cur[i])<0)
                {
                    delta[i] = std::max(delta_prev[i]*n_neg,delta_min);
                    if (cost_cur[i] > cost_prev[i])
                    {
                        w_prev.at<float>(i) = w.at<float>(i);
                        w.at<float>(i) = w.at<float>(i)-sign(grad_prev[i])*delta_prev[i];
                    }
                    grad_prev[i] = 0;
                } else if ((grad_prev[i]*grad_cur[i])==0)
                {
                    w_prev.at<float>(i) = w.at<float>(i);
                    w.at<float>(i) = w.at<float>(i) - sign(grad_prev[i])*delta[i];
                    grad_prev[i] = grad_cur[i];
                }
            }
            

            cost_prev_x = cost_cur_x;
            cost_prev_y = cost_cur_y;
            cost_prev_z = cost_cur_z;
            cost_prev_yaw = cost_cur_yaw;

            cost_prev[0] = cost_prev_x;
            cost_prev[1] = cost_prev_y;
            cost_prev[2] = cost_prev_z;
            cost_prev[3] = cost_prev_yaw;

            
        }
        return w;
    }
    

    void setGoal(cv::Point3f statePt)
    {
        goal_x = (float)statePt.x;
        goal_y = (float)statePt.y;
        goal_z = (float)statePt.z;
        goal_yaw = (float)(atan2(goal_y-pose_y,goal_x-pose_x));
        searchAngle = goal_yaw;
        master_pose = (cv::Mat_<float>(4,1) << goal_x,goal_y,goal_z,goal_yaw);
    }

    void moveDrone(cv::Mat w)
    {
        // MRS - waypoint --------------------------------------
        srv.request.header.stamp = ros::Time::now();

        srv.request.reference.position.x = w.at<float>(0);
        srv.request.reference.position.y = w.at<float>(1);
        srv.request.reference.position.z = w.at<float>(2);
        srv.request.reference.heading    = w.at<float>(3); 
        
        if (client.call(srv))
        {
            ROS_INFO("Successfull calling service\n");
            sleep(CONTROLLER_PERIOD);
        }
        else 
        {
            ROS_ERROR("Could not publish\n");
        }
        //---------------------------------------------------------------

    }

    void callback_three(const OdometryConstPtr obj,const OdometryConstPtr obj_secondary, const OdometryConstPtr obj_third, const OdometryConstPtr pose,const EstimatedStateConstPtr& yaw)
    {
        ROS_INFO_STREAM("Synchronized");
        //------------MEASUREMENTS------------------------
        state = (cv::Mat_<float>(4,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,yaw->state[0]);
        state_cov = SensFuseThree::convertToCov(pose);
            
        pose_x = (float)(pose->pose.pose.position.x);
        pose_y = (float)(pose->pose.pose.position.y);

        if (count < 1)
        {
            init_x = pose_x;
            init_y = pose_y;
        }
        
        
        // if I see...
        if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x=='\0')) && (obj_third->pose.pose.position.x=='\0'))
        {
            ROS_INFO_STREAM("Only I see...\n");
            cv::Point3f statePt = SensFuseThree::updateKFSingle(obj);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);

        }
        // if second sees...
        else if (((obj->pose.pose.position.x=='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x=='\0'))
        
        {
            ROS_INFO_STREAM("Only secondary sees...\n");
            cv::Point3f statePt = SensFuseThree::updateKFSingle(obj_secondary);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);
        }   
         // if third sees...
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x=='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            ROS_INFO_STREAM("Only third sees...\n");
            cv::Point3f statePt = SensFuseThree::updateKFSingle(obj_third);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);

        }  
        // if 1 & 2 sees
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x=='\0'))
        
        {
            ROS_INFO_STREAM("I and secondary see...\n");
            cv::Point3f statePt = SensFuseThree::updateKFDouble(obj, obj_secondary);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);

        } 
        // if 1 & 3 sees
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x=='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            ROS_INFO_STREAM("I and third sees...\n");
            cv::Point3f statePt = SensFuseThree::updateKFDouble(obj, obj_third);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);
        }    
         // if 2 & 3 sees
        else if (((obj->pose.pose.position.x=='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            ROS_INFO_STREAM("Only secondary and third see...\n");
            cv::Point3f statePt = SensFuseThree::updateKFDouble(obj_secondary, obj_third);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);
        }  
         // if all 3 see...
        else if (((obj->pose.pose.position.x!='\0') && (obj_secondary->pose.pose.position.x!='\0')) && (obj_third->pose.pose.position.x!='\0'))
        
        {
            ROS_INFO_STREAM("All see...\n");
            cv::Point3f statePt = SensFuseThree::updateKFTriple(obj,obj_secondary, obj_third);
            //---------------------------------------------------------------
            SensFuseThree::setGoal(statePt);
            w = SensFuseThree::calculateFormation(state, master_pose, state_cov, obj_cov);
            SensFuseThree::moveDrone(w);
        } 
        else
        {
            // SEARCH--------------------------------------------------------
            ROS_INFO_STREAM("I search...\n");
            searchAngle += M_PI/SEARCH_SIZE;
            goal_x = pose_x;
            goal_y = pose_y;
            goal_z = heights[height_count];
                
            height_count++;
            if (height_count>=sizeof(heights)/sizeof(heights[0]))
            {
                height_count = 0;
            }

            goal_yaw = yaw->state[0]+searchAngle;
            // SEARCH--------------------------------------------------------
            w = (cv::Mat_<float>(4,1) << goal_x,goal_y,goal_z,goal_yaw);
            SensFuseThree::moveDrone(w);

        } 
        count++;
    }
};


int main(int argc, char** argv)
{
    ROS_INFO("Sensor Fusion node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_sensor_fusion";


    if(argc < 4) 
    {
        std::cerr<<"Please, enter the drone position around the goal, in the following form: UAV_NAME_OWN UAV_NAME_SEC UAV_NAME_THIRD x y z "<<'\n';
        return 1; 
    }


    std::istringstream              source_cmd_x(argv[4]);
    std::istringstream              source_cmd_y(argv[5]);
    std::istringstream              source_cmd_z(argv[6]);

    float offset_parameter_x;
    float offset_parameter_y;
    float offset_parameter_z;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    SensFuseThree sf(nh,argv[1],argv[2],argv[3],offset_parameter_x,offset_parameter_y,offset_parameter_z);

    ros::spin();

    return 0;
}
