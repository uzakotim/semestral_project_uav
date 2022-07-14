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
#define CALCULATION_STEPS 300 //150
#define CALCULATIOM_STEPS_IN_MOTION 60 //30
#define RADIUS 3.0
#define FLYING_AROUND 0
#define SEARCH_SIZE 8

#define RATE 1000
#define NUMBER_OF_TRACKER_COUNT 22.0

class Formation
{
public:
    ros::NodeHandle nh;   
    message_filters::Subscriber<Odometry>       sub_1;
    message_filters::Subscriber<EstimatedState> sub_2;
    message_filters::Subscriber<Odometry>       sub_3;

    typedef sync_policies::ApproximateTime<Odometry,EstimatedState,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    
    cv::Point3f                 initial_state    = cv::Point3f(0,0,1);
    //------------TOPICS------------------------------
    //subscribers
    std::string                 sub_pose_topic   = "";
    std::string                 sub_object_topic = "";
    std::string                 sub_yaw_topic = "";
    std::string                 sub_detection_topic = "";
    //publishers
    std::string                 pub_pose_topic   = "";

    //------------MODE--PARAMETER-----------------------
    int yawSearch {1}; // 1 - ON ; 0 - OFF;


    //------------OPTIMIZATION-PARAMETERS----------------`
    float                      center_x {0.0};
    float                      center_y {0.0};
    float                      center_z {0.0};
    
    float                      x_previous;
    float                      y_previous;
    float                      z_previous;

    
    float                      offset_x;
    float                      offset_y;
    float                      offset_z;

    float angle = 0.0;
    float searchAngle = -M_PI;

    float radius = RADIUS;
    
    float goal_x {0.0};
    float goal_y {0.0};
    float goal_z {0.0};
    float goal_yaw{0.0};

    float pose_x {0.0};
    float pose_y {0.0};

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
    
    cv::Mat w_prev = (cv::Mat_<float>(3,1) <<  0,0,0,0);
    cv::Mat w;
    cv::Mat master_pose;
    
    cv::Mat state,state_cov,obj_cov;

    cv::Mat tracker_vector = (cv::Mat_<float>(3,1) << 0,0,0,0);
    
    std::vector<cv::Mat> tracker;

    float resulting_cost_x {0};
    float resulting_cost_y {0};
    float resulting_cost_z {0};
    float resulting_cost_yaw {0};

    // ---------OUTPUT MSG-----------------------------------
    boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
    ros::ServiceClient client;
    mrs_msgs::Vec4 srv;

    Formation(char** argv, float x_parameter,float y_parameter, float z_parameter)
    {
        //------------TOPICS-DECLARATIONS----------------

        sub_pose_topic  +="/";
        sub_pose_topic  +=argv[4];
        sub_pose_topic  +="/odometry/odom_main/";

        sub_yaw_topic  +="/";
        sub_yaw_topic  +=argv[4];
        sub_yaw_topic  +="/odometry/heading_state_out/";

        sub_detection_topic += "/";
        sub_detection_topic += argv[4];
        sub_detection_topic += "/tracker/";

        pub_pose_topic+="/";
        pub_pose_topic+=argv[4];
        pub_pose_topic+="/control_manager/goto";


        ROS_INFO_STREAM(pub_pose_topic);

        sub_1.subscribe(nh,sub_pose_topic,1);
        sub_2.subscribe(nh,sub_yaw_topic,1);
        sub_3.subscribe(nh,sub_detection_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2,sub_3));
        sync->registerCallback(boost::bind(&Formation::callback,this,_1,_2,_3));
                
        client = nh.serviceClient<mrs_msgs::Vec4>(pub_pose_topic);

        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;

        ROS_INFO("Leader Controller Node Initialized Successfully"); 
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

    float CostYaw(cv::Mat w,cv::Mat w_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov)
    {
        // resulting_cost_z = CONTROL_GAIN_STATE*std::pow((w.at<float>(2) - w_prev.at<float>(2)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2);  
        resulting_cost_yaw = CONTROL_GAIN_STATE*std::pow((w.at<float>(3) - w_prev.at<float>(3)),2) + CONTROL_GAIN_GOAL*std::pow((w.at<float>(3) - (master_pose.at<float>(3))),2) + 0.5*cv::determinant(state_cov)*10e-20 + 0.5*cv::determinant(object_cov)*10e-10;  
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
        // run optimization
        // costs
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


    void callback(const OdometryConstPtr& pose, const EstimatedStateConstPtr& yaw, const OdometryConstPtr obj)
    {
        ROS_INFO("Synchronized\n");
        /*-------------MEASUREMENTS----------------*/
        pose_x = (float)(pose->pose.pose.position.x);
        pose_y = (float)(pose->pose.pose.position.y);

        obj_x = (float)(obj->pose.pose.position.x);
        obj_y = (float)(obj->pose.pose.position.y);
        obj_z = (float)(obj->pose.pose.position.z);
        obj_yaw = (float)(std::round(atan2(obj_y-pose_y,obj_x-pose_x)));
        obj_cov = Formation::convertToCov(obj);
        
        master_pose = (cv::Mat_<float>(4,1) << obj_x,obj_y,obj_z,obj_yaw);
        state = (cv::Mat_<float>(4,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,yaw->state[0]);


        state_cov = Formation::convertToCov(pose);
        //------------RPROP----------------
        // goal-driven behaviour
        w = (cv::Mat_<float>(4,1)<< state.at<float>(0),state.at<float>(1),state.at<float>(2),state.at<float>(3)); 
        ROS_INFO_STREAM("master at"<<master_pose);
        // run optimization
        w = Formation::calculateFormation(w, master_pose,state_cov,obj_cov);

        
        
        // moving around origin -------------------------     
        if (FLYING_AROUND == 1)
        {
            angle += M_PI/32;

            if (angle >= 2*M_PI)
            {
                angle = 0;
            }
            goal_x = 0 + radius * cos(angle);
            goal_y = 0 + radius * sin(angle);
            goal_z = 3.0;
            // goal_x = center_x;
            // goal_y = center_y;
            // ROS_INFO_STREAM("angle: "<<angle<<" radius: "<<radius<<'\n');
            ROS_INFO_STREAM("goal x: "<<goal_x<<" goal y: "<<goal_y<<'\n');
        }
        

        //---------------------------------------------------



        if (FLYING_AROUND==0)
        {
            goal_x = w.at<float>(0);
            goal_y = w.at<float>(1);
            goal_z = w.at<float>(2);
        }

        if (obj->pose.pose.position.x == '\0')
        {
            // If I don't see, I search
            
            searchAngle += M_PI/SEARCH_SIZE;
            ROS_INFO_STREAM("[search]: ["<<searchAngle<<"]");

            if (searchAngle >= M_PI)
            {
                searchAngle = -M_PI;
            }
            
            goal_yaw = searchAngle;

        }else
        {
            goal_yaw = w.at<float>(3);
            searchAngle = goal_yaw;
        }


        srv.request.goal = boost::array<float, 4>{goal_x,goal_y,goal_z,goal_yaw};
      
        if (client.call(srv))
        {
            ROS_INFO("Successfull calling service\n");
            sleep(CONTROLLER_PERIOD);
        }
        else 
        {
            ROS_ERROR("Could not publish\n");
        }
        // Optionally publish error
        
    }

};



int main(int argc, char** argv)
{
    
    if(argv[4] == NULL) 
    {
        std::cerr<<"Please, enter the drone trajectory center, in the following form: x y z UAV_NAME "<<'\n';
        return 1; 
    }
    std::istringstream              source_cmd_x(argv[1]);
    std::istringstream              source_cmd_y(argv[2]);
    std::istringstream              source_cmd_z(argv[3]);
    
    float offset_parameter_x;
    float offset_parameter_y;
    float offset_parameter_z;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;
 

    ROS_INFO_STREAM  ("Instanciating Leader Controller\n");
    std::string node_name = "";
    node_name += argv[4];
    node_name += "_formation_leader";
    
    ros::init(argc, argv, node_name);
    Formation fc(argv,offset_parameter_x,offset_parameter_y,offset_parameter_z);
    ros::spin();

    return 0;
}
