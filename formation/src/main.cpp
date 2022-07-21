// Copyright [2021] [Timur Uzakov]
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/EstimatedState.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>
#include "ros/service_client.h"

#include <iostream>
#include <cmath>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace mrs_msgs;

#define CONTROLLER_PERIOD 1
#define DELTA_MAX 0.5
#define CONTROL_GAIN_GOAL 20
#define CONTROL_GAIN_STATE 1
#define NUMBER_OF_TRACKER_COUNT 22.0
#define CALCULATION_STEPS 150
#define CALCULATIOM_STEPS_IN_MOTION 30
#define RADIUS 2.0

class Formation
{
public:
    ros::NodeHandle nh;   
    // message_filters::Subscriber<Odometry>     sub_1;
    message_filters::Subscriber<Odometry>     sub_1;
    message_filters::Subscriber<EstimatedState> sub_2;

    typedef sync_policies::ApproximateTime<Odometry,EstimatedState> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    
    cv::Point3f                 initial_state    = cv::Point3f(0,0,1);
    //------------TOPICS------------------------------
    //subscribers
    std::string                 sub_pose_topic   = "";
    std::string                 sub_object_topic = "";
    std::string                 sub_yaw_topic = "";
    //publishers
    std::string                 pub_pose_topic   = "";


    //------------OPTIMIZATION-PARAMETERS----------------

    int count {0};
    cv::Mat state,state_cov,object_coord,object_cov;
    cv::Mat tracker_vector = (cv::Mat_<float>(3,1) << 0,0,0);
    std::vector<cv::Mat> tracker;
    
    cv::Mat w_prev = (cv::Mat_<float>(3,1) <<  0,0,0);
    cv::Mat w;
    cv::Mat master_pose;

    // parameters
    float n_pos {1.2};
    float n_neg {0.5};
    float delta_max {DELTA_MAX};
    float delta_min {0.001}; // 0.000001
   
    float cost_prev_x{0},cost_cur_x{0};
    float cost_prev_y{0},cost_cur_y{0};
    float cost_prev_z{0},cost_cur_z{0};
    float cost_dif_x{0};
    float cost_dif_y{0};
    float cost_dif_z{0};
    float step_x{0};
    float step_y{0};
    float step_z{0};

    

    std::vector<float> cost_cur;
    std::vector<float> cost_prev;
    std::vector<float> grad_cur {0,0,0}; // 0 0 0 
    std::vector<float> grad_prev {0,0,0}; // 0 0 0

    std::vector<float> delta {0.5,0.5,0.5};
    std::vector<float> delta_prev {0.5,0.5,0.5};

    int k{0};  //computing steps
    
    float                      x_previous;
    float                      y_previous;
    float                      z_previous;

    
    float                      offset_x;
    float                      offset_y;
    float                      offset_z;

    float resulting_cost_x {0};
    float resulting_cost_y {0};
    float resulting_cost_z {0};


    //-------------------------------------------------------
    float angle {0.0};
    float theta {0.0};
    
    float goal_x{0.0};
    float goal_y{0.0};
    float goal_z{0.0};

    int   around {0};

    
    // ---------OUTPUT MSG-----------------------------------
    boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
    ros::ServiceClient client;
    mrs_msgs::Vec4 srv;
    //--------  COST FUNCTIONS ------------------------------
    float CostX(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,float offset_x)
    {        
        // resulting_cost_x = CONTROL_GAIN_STATE*std::pow((w.at<float>(0) - w_prev.at<float>(0)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(0) - (master_pose.at<float>(0)+offset_x)),2); //-10 -4  
        resulting_cost_x = CONTROL_GAIN_STATE*std::pow((w.at<float>(0) - w_prev.at<float>(0)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(0) - (master_pose.at<float>(0)+offset_x)),2) + 0.5*cv::determinant(state_cov)*10e-20 + 0.5*cv::determinant(object_cov)*10e-10; //-10 -4  
        return resulting_cost_x;
    }
    float CostY(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,float offset_y)
    {
        // resulting_cost_y = CONTROL_GAIN_STATE*std::pow((w.at<float>(1) - w_prev.at<float>(1)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(1) - (master_pose.at<float>(1)+offset_y)),2);  
        resulting_cost_y = CONTROL_GAIN_STATE*std::pow((w.at<float>(1) - w_prev.at<float>(1)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(1) - (master_pose.at<float>(1)+offset_y)),2) + 0.5*cv::determinant(state_cov)*10e-20 + 0.5*cv::determinant(object_cov)*10e-10;  
        return resulting_cost_y;
    }
    float CostZ(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,float offset_z)
    {
        // resulting_cost_z = CONTROL_GAIN_STATE*std::pow((w.at<float>(2) - w_prev.at<float>(2)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2);  
        resulting_cost_z = CONTROL_GAIN_STATE*std::pow((w.at<float>(2) - w_prev.at<float>(2)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2) + 0.5*cv::determinant(state_cov)*10e-20 + 0.5*cv::determinant(object_cov)*10e-10;  
        return resulting_cost_z;
    }
    int sign(float x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }


    Formation(char** argv, float x_parameter,float y_parameter, float z_parameter, int around_parameter)
    {
        //------------TOPICS-DECLARATIONS----------------
        if (argv[5] != NULL) 
        {
            sub_object_topic+="/";
            sub_object_topic+=argv[4];
            sub_object_topic+="/goal/";
        } else
        {
            sub_object_topic+="/";
            sub_object_topic+=argv[4];
            sub_object_topic+="/tracker/";
        }

        sub_pose_topic  +="/";
        sub_pose_topic  +=argv[4];
        sub_pose_topic  +="/odometry/odom_main/";

        sub_yaw_topic  +="/";
        sub_yaw_topic  +=argv[4];
        sub_yaw_topic  +="/odometry/heading_state_out/";

        pub_pose_topic+="/";
        pub_pose_topic+=argv[4];
        pub_pose_topic+="/control_manager/goto/";

        // sub_1.subscribe(nh,sub_object_topic,1);
        sub_1.subscribe(nh,sub_pose_topic,1);
        sub_2.subscribe(nh,sub_yaw_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&Formation::callback,this,_1,_2));
                
        client = nh.serviceClient<mrs_msgs::Vec4>(pub_pose_topic);
        
         // Gradient Descent Parameters
        x_previous = initial_state.x;
        y_previous = initial_state.y;
        z_previous = initial_state.z;

        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;

        around = around_parameter;

        ROS_INFO("Motion Red Drone Controller Node Initialized Successfully"); 
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
    void callback(const OdometryConstPtr& pose, const EstimatedStateConstPtr& yaw)
    {

        
        

        if (around == 1)
        {
            angle = M_PI/2;

            theta += M_PI/32;

            if (theta >= 2*M_PI)
            {
                theta = 0;
            }
            goal_x = offset_x + RADIUS * cos(theta);
            goal_y = offset_y + RADIUS * sin(theta);
            goal_z = offset_z;
        }
        else
        {
            angle += M_PI/8;
            if (angle >= 2*M_PI)
            {
                angle = 0.0;
            }

            goal_x = offset_x;
            goal_y = offset_y;
            goal_z = offset_z;
        }
        ROS_INFO_STREAM("goal x: "<<goal_x<<" goal y: "<<goal_y<<'\n');
        // ROS_INFO("Synchronized\n");
        srv.request.goal = boost::array<float, 4>{goal_x,goal_y,goal_z,angle};

        if (client.call(srv))
        {
            ROS_INFO("Successfull calling service\n");
            sleep(CONTROLLER_PERIOD);
        }
        else 
        {
                ROS_ERROR("Could not publish\n");
        }
    }

};



int main(int argc, char** argv)
{
    
    if(argv[4] == NULL) 
    {
        std::cerr<<"Please, enter the drone position around the goal, in the following form: x y z UAV_NAME ..."<<'\n';
        return 1; 
    }
    std::istringstream              source_cmd_x(argv[1]);
    std::istringstream              source_cmd_y(argv[2]);
    std::istringstream              source_cmd_z(argv[3]);
    std::istringstream              source_cmd_around(argv[5]);
    
    float offset_parameter_x;
    float offset_parameter_y;
    float offset_parameter_z;
    int   around_parameter;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;
    if(!(source_cmd_around>>around_parameter)) return 1;
 

    ROS_INFO_STREAM  ("Instanciating Red Drone Motion Controller\n");
    std::string node_name = "";
    node_name += argv[4];
    node_name += "_formation_controller";
    
    ros::init(argc, argv, node_name);
    Formation fc(argv,offset_parameter_x,offset_parameter_y,offset_parameter_z,around_parameter);
    ros::spin();

    return 0;
}
