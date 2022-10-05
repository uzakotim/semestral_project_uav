// Copyright [2022] [Timur Uzakov]
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/sync_policies/exact_time.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/EstimatedState.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>

#include <nav_msgs/Odometry.h>
#include <numeric>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
// include ros library
#include <ros/ros.h>
#include "ros/service_client.h"

#include <sensor_msgs/Image.h>
#include <string.h>
#include <vector>
#include <queue>





// ---------------------MACROS--------------------------------------

#define CALCULATION_STEPS 10 //10
#define CALCULATIOM_STEPS_IN_MOTION 5 //5

#define CONTROLLER_PERIOD 0.1

// the most optimal value is 1, the less the faster motion
#define CONTROL_GAIN_GOAL 300 //200
#define CONTROL_GAIN_DETECTION 0.001 //200
#define CONTROL_GAIN_STATE_ESTIMATION 0.001 //200
#define CONTROL_GAIN_STATE 0.1  // 1
#define CONTROL_GAIN_STATE_Z 0.01 // 100
// influences how sharp are drone's motions - the lower the sharper

#define DELTA_MAX 0.5 //0.5
// determines how fast drone optimises - the smaller the faster
#define SEARCH_SIZE 8
#define SEARCH_HEIGHT 3.0
#define PRINT_OUT 1

// -----------------------------------------------------------------
using namespace geometry_msgs;
using namespace message_filters;
using namespace mrs_msgs;
using namespace nav_msgs;
using namespace sensor_msgs;

class SensFuseTwo
{

private:    
    
    ros::NodeHandle nh;
    ros::Publisher goal_pub;
    
    message_filters::Subscriber<PoseWithCovarianceArrayStamped> obj_sub;
    message_filters::Subscriber<PoseWithCovarianceArrayStamped> obj_secondary_sub;

    message_filters::Subscriber<Odometry> pose_sub;
    message_filters::Subscriber<EstimatedState> yaw_sub;


    // typedef sync_policies::ApproximateTime<PoseWithCovarianceArrayStamped,PoseWithCovarianceArrayStamped,Odometry,EstimatedState> MySyncPolicy;
    // typedef Synchronizer<MySyncPolicy> Sync;
    // boost::shared_ptr<Sync> sync;

    typedef sync_policies::ApproximateTime<PoseWithCovarianceArrayStamped,PoseWithCovarianceArrayStamped> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


    std::string sub_obj_topic           = "";
    std::string sub_obj_topic_secondary = "";

    std::string sub_pose_topic          = "";
    std::string sub_yaw_topic           = "";

    std::string pub_pose_topic          = "";
    std::string pub_goal_topic          = "";

public:
// -----------------------------------------------------------------

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
    mrs_msgs::ReferenceStampedSrv srv;

    //---------------------------------------------------------
    //------------------------------PARAMETERS-----------------
    float                      offset_x;
    float                      offset_y;
    float                      offset_z;
    float                      offset_angle;

    float goal_x {0.0};
    float goal_y {0.0};
    float goal_z {0.0};
    float goal_yaw{0.0};
    
    float pose_x {0.0};
    float pose_y {0.0};
    float pose_z {0.0};

    float init_x {0.0};
    float init_y {0.0};
    float init_z {0.0};
    // ----------Formation controller parameters--------------
    const float n_pos {1.2};
    const float n_neg {0.5};
    const float delta_max {DELTA_MAX};
    const float delta_min {0.001}; // 0.000001
   
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

    size_t k{0};  //computing steps
    u_int64_t count {0};
    size_t height_count {0};
    
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
    

    float cov_avg;
    double max_radius;

    cv::Point3f prevState;
    //-----------------------------------------------------------------------------------------------------------------

    SensFuseTwo(char* name_main, char* name_secondary,float angle_parameter,float z_parameter)
    {   

    // ---------------------INITIALIZATION--------------------------------------
        //subscribers

        sub_obj_topic  += "/";
        sub_obj_topic  += name_main;
        sub_obj_topic  += "/points/";

        sub_obj_topic_secondary  += "/";
        sub_obj_topic_secondary  += name_secondary;
        sub_obj_topic_secondary  += "/points/";

        sub_yaw_topic  += "/";
        sub_yaw_topic  += name_main;
        sub_yaw_topic  += "/odometry/heading_state_out/";

        sub_pose_topic += "/";
        sub_pose_topic += name_main;
        sub_pose_topic += "/odometry/odom_main/";

        obj_sub.subscribe(nh,sub_obj_topic,1);
        obj_secondary_sub.subscribe (nh,sub_obj_topic_secondary,1);
        pose_sub.subscribe(nh,sub_pose_topic,1);
        yaw_sub.subscribe(nh,sub_yaw_topic,1);
        // publishers

        pub_pose_topic += "/";
        pub_pose_topic += name_main;
        pub_pose_topic += "/control_manager/reference";

        // ------------------------------------------------------------------- 
        // sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub,pose_sub,yaw_sub));
        // sync->registerCallback(boost::bind(&SensFuseTwo::callback_two,this, _1,_2,_3,_4));
        
        sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub));
        sync->registerCallback(boost::bind(&SensFuseTwo::callback_two,this, _1,_2));
        
        
        
        client = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>(pub_pose_topic);
        // -------------------------------------------------------------------
        // Taking parameters to set robot position
        offset_angle = angle_parameter;
        offset_z = z_parameter;
        //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(6,6) <<  1,0,0,1,0,0,
                                                        0,1,0,0,1,0,
                                                        0,0,1,0,0,1,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        
        KF.statePre.at<float>(0) = 0;
        KF.statePre.at<float>(1) = 0;
        KF.statePre.at<float>(2) = 0;
        KF.statePre.at<float>(3) = 0;
        KF.statePre.at<float>(4) = 0;
        KF.statePre.at<float>(5) = 0;

        // was previously at initialization and set to Scalar(0)
        measurement.setTo(cv::Scalar(0)); 

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1)); //Q
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10)); //R
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----
        obj_cov             = (cv::Mat_<float>(6,6) <<  1,0,0,0,0,0,
                                                        0,1,0,0,0,0,
                                                        0,0,1,0,0,0,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);


        ROS_INFO("[Two Drones Down] All functions initialized");
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
    }

    void SetMeasurement(const cv::Point3f center)
    {
        measurement.at<float>(0) = center.x;
        measurement.at<float>(1) = center.y;
        measurement.at<float>(2) = center.z;
    }

    cv::Point3f UpdateKalmanFilter(cv::Mat_<float>  measurement)
    {
        cv::Mat       estimated = KF.correct(measurement);
        cv::Point3f   statePt(estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
        cv::Point3f   measPt(measurement(0),measurement(1),measurement(2));
        return      statePt;
    }

    //--------  COST FUNCTIONS ------------------------------
    float CostX(cv::Mat w,cv::Mat w_prev,cv::Mat master_pose,cv::Mat state_cov,float object_cov,float offset_x)
    {        
        resulting_cost_x = CONTROL_GAIN_STATE*std::pow((w.at<float>(0) - w_prev.at<float>(0)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(0) - (master_pose.at<float>(0)+offset_x)),2) + CONTROL_GAIN_STATE_ESTIMATION*cv::determinant(state_cov)*10e-6 + CONTROL_GAIN_DETECTION*object_cov; //-10 -4  
        return resulting_cost_x;
    }
    float CostY(cv::Mat w,cv::Mat w_prev,cv::Mat master_pose,cv::Mat state_cov,float object_cov,float offset_y)
    {
        resulting_cost_y = CONTROL_GAIN_STATE*std::pow((w.at<float>(1) - w_prev.at<float>(1)),2)  + CONTROL_GAIN_GOAL*std::pow((w.at<float>(1) - (master_pose.at<float>(1)+offset_y)),2) + CONTROL_GAIN_STATE_ESTIMATION*cv::determinant(state_cov)*10e-6 + CONTROL_GAIN_DETECTION*object_cov;  
        return resulting_cost_y;
    }
    float CostZ(cv::Mat w,cv::Mat w_prev,cv::Mat master_pose,cv::Mat state_cov,float object_cov,float offset_z)
    {
        resulting_cost_z = CONTROL_GAIN_STATE_Z*std::pow((w.at<float>(2) - w_prev.at<float>(2)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2) + CONTROL_GAIN_STATE_ESTIMATION*cv::determinant(state_cov)*10e-6 + CONTROL_GAIN_DETECTION*object_cov;  
        return resulting_cost_z;
    }

    float CostYaw(cv::Mat w,cv::Mat w_prev,cv::Mat master_pose,cv::Mat state_cov,float object_cov)
    {
        resulting_cost_yaw = CONTROL_GAIN_STATE*std::pow((w.at<float>(3) - w_prev.at<float>(3)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(3) - (master_pose.at<float>(3))),2) + CONTROL_GAIN_STATE_ESTIMATION*cv::determinant(state_cov)*10e-6  + CONTROL_GAIN_DETECTION*object_cov;  
        return resulting_cost_yaw;
    }
    int sign(float x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

    
    cv::Mat calculateFormation( cv::Mat w,cv::Mat master_pose,cv::Mat state_cov,float object_cov)
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
        if ((std::abs(w.at<float>(0) - w_prev.at<float>(0))<0.2) || (std::abs(w.at<float>(1) - w_prev.at<float>(1))<0.2) || (std::abs(w.at<float>(2) - w_prev.at<float>(2))<0.2) || (std::abs(w.at<float>(3) - w_prev.at<float>(3))<0.2))
        {
                k = CALCULATION_STEPS;
        } else  k = CALCULATIOM_STEPS_IN_MOTION;
        // -------------------------------------------------- 
    
        // k = CALCULATION_STEPS;

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
                    w.at<float>(i) = w.at<float>(i) - SensFuseTwo::sign(grad_cur[i])*delta[i];
                    grad_prev[i] = grad_cur[i]; 
                } else if ((grad_prev[i]*grad_cur[i])<0)
                {
                    delta[i] = std::max(delta_prev[i]*n_neg,delta_min);
                    if (cost_cur[i] > cost_prev[i])
                    {
                        w_prev.at<float>(i) = w.at<float>(i);
                        w.at<float>(i) = w.at<float>(i)-SensFuseTwo::sign(grad_prev[i])*delta_prev[i];
                    }
                    grad_prev[i] = 0;
                } else if ((grad_prev[i]*grad_cur[i])==0)
                {
                    w_prev.at<float>(i) = w.at<float>(i);
                    w.at<float>(i) = w.at<float>(i) - SensFuseTwo::sign(grad_prev[i])*delta[i];
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
            // ROS_INFO("Successfull calling service\n");
            sleep(CONTROLLER_PERIOD);
        }
        else 
        {
            ROS_ERROR("Could not publish\n");
        }
        //---------------------------------------------------------------

    }

    double getAverage(std::vector<double> v)
    {
        return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    }
    
    // void callback_two(PoseWithCovarianceArrayStampedConstPtr obj,PoseWithCovarianceArrayStampedConstPtr obj_secondary, OdometryConstPtr pose,EstimatedStateConstPtr yaw)
    void callback_two(PoseWithCovarianceArrayStampedConstPtr obj,PoseWithCovarianceArrayStampedConstPtr obj_secondary)
    {
        if (PRINT_OUT == 1)
            // ROS_INFO_STREAM("[SYNC]: PoseWithCovarianceArrayStampedConstPtr obj,PoseWithCovarianceArrayStampedConstPtr obj_secondary, OdometryConstPtr pose,EstimatedStateConstPtr yaw");
            ROS_INFO_STREAM("[Synchronization successful]");

        // cv::Point3f predictPt = PredictUsingKalmanFilter();
        //------------MEASUREMENTS------------------------    
        /*
        pose_x = (float)(pose->pose.pose.position.x);
        pose_y = (float)(pose->pose.pose.position.y);
        pose_z = (float)(pose->pose.pose.position.z);
        
        state = (cv::Mat_<float>(4,1) << pose_x,pose_y,pose_z,yaw->state[0]);
        state_cov = SensFuseTwo::convertToCov(pose);

        if (PRINT_OUT == 1)
            ROS_INFO_STREAM("[STATE]: "<<"\nx: "<<state.at<float>(0)<<"\ny: "<<state.at<float>(1)<<"\nz: "<<state.at<float>(2));
        if (count < 1)
        {
            init_x = pose_x;
            init_y = pose_y;

            prevState.x = init_x;
            prevState.y = init_y;
            prevState.z = 0.5;

            
        }


        std::vector<double> all_x,all_y,all_z,all_cov;
        std::priority_queue<double> all_radius;
        for (PoseWithCovarianceIdentified point : obj->poses)
        {
            all_x.push_back(point.pose.position.x);
            all_y.push_back(point.pose.position.y);
            all_z.push_back(point.pose.position.z);
            all_cov.push_back(point.pose.orientation.w);
        }
        for (PoseWithCovarianceIdentified point : obj_secondary->poses)
        {
            all_x.push_back(point.pose.position.x);
            all_y.push_back(point.pose.position.y);
            all_z.push_back(point.pose.position.z);
            all_cov.push_back(point.pose.orientation.w);
        }

        if (all_x.size()==0)
        {
            center3D.x = prevState.x;
            center3D.y = prevState.y;
            center3D.z = prevState.z;
            cov_avg = 1000;
            max_radius = 3.0;
        }
        else
        {
            center3D.x = SensFuseTwo::getAverage(all_x);
            center3D.y = SensFuseTwo::getAverage(all_y);
            center3D.z = SensFuseTwo::getAverage(all_z);
            cov_avg = SensFuseTwo::getAverage(all_cov);
            
            for (PoseWithCovarianceIdentified point : obj->poses)
            {
                double value = std::sqrt(std::pow(point.pose.position.x-center3D.x,2) + std::pow(point.pose.position.y-center3D.y,2));
                all_radius.push(value);
            }
            for (PoseWithCovarianceIdentified point : obj_secondary->poses)
            {
                double value = std::sqrt(std::pow(point.pose.position.x-center3D.x,2) + std::pow(point.pose.position.y-center3D.y,2));
                all_radius.push(value);
            }
            max_radius = all_radius.top();
            if (max_radius<3.0){
                max_radius = 3.0;
            }else
            {
                max_radius = 2.0*all_radius.top()/3.0;
            }
        }
        
        SetMeasurement(center3D);

        cv::Point3f statePt = UpdateKalmanFilter(measurement);

        if (PRINT_OUT == 1)
        {
            ROS_INFO_STREAM("[CENTER]:\nx: "<<center3D.x<<"\ny: "<<center3D.y<<"\nz: "<<center3D.z);
            ROS_INFO_STREAM("[RADIUS]: "<<max_radius);
        }
        
        goal_yaw = (float)(atan2(statePt.y-pose_y,statePt.x-pose_x));
        
        master_pose = (cv::Mat_<float>(4,1) << statePt.x,statePt.y,statePt.z,goal_yaw);
        offset_x = max_radius*cos(offset_angle);
        offset_y = max_radius*sin(offset_angle);

        w = SensFuseTwo::calculateFormation(state, master_pose, state_cov, cov_avg);
        if (PRINT_OUT == 1)
        {
            ROS_INFO_STREAM("[WAYPOINT]:\nx: "<<w.at<float>(0)<<"\ny: "<<w.at<float>(1)<<"\nz: "<<w.at<float>(2)<<"\nyaw: "<<w.at<float>(3));
        }
        SensFuseTwo::moveDrone(w);
        
        prevState.x = statePt.x;
        prevState.y = statePt.y;
        prevState.z = statePt.z;
        */
        count++;
        if (count>100)
            count = 2;
    }
};


int main(int argc, char** argv)
{
    ROS_INFO("Sensor Fusion Two Down node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_sensor_fusion_down";


    if(argc < 4) 
    {
        std::cerr<<"Please, enter the drone position's angle on circle around centroid, in the following form: UAV_NAME_OWN UAV_NAME_SEC angle z"<<'\n';
        return 1; 
    }


    std::istringstream              source_cmd_angle(argv[3]);
    std::istringstream              source_cmd_z(argv[4]);

    float offset_parameter_angle;
    float offset_parameter_z;

    if(!(source_cmd_angle>>offset_parameter_angle)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;

    ros::init(argc, argv, node_name);
    char * first  = argv[1];
    char * second = argv[2];
    SensFuseTwo sf(first,second,offset_parameter_angle,offset_parameter_z);
    ros::spin();

    return 0;
}
