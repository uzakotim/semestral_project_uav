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

#define CONTROLLER_PERIOD 0.1

class Formation
{
public:
    ros::NodeHandle nh;   
    message_filters::Subscriber<Odometry>     sub_1;
    message_filters::Subscriber<Odometry>     sub_2;
    message_filters::Subscriber<EstimatedState> sub_3;

    typedef sync_policies::ApproximateTime<Odometry, Odometry,EstimatedState> MySyncPolicy;
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
    double n_pos {1.2};
    double n_neg {0.5};
    double delta_max {0.5};
    double delta_min {0.000001};
   
    double cost_prev_x{0},cost_cur_x{0};
    double cost_prev_y{0},cost_cur_y{0};
    double cost_prev_z{0},cost_cur_z{0};
    double cost_dif_x{0};
    double cost_dif_y{0};
    double cost_dif_z{0};
    double step_x{0};
    double step_y{0};
    double step_z{0};


    std::vector<double> cost_cur;
    std::vector<double> cost_prev;
    std::vector<double> grad_cur {0,0,0};
    std::vector<double> grad_prev {0,0,0};

    std::vector<double> delta {0.5,0.5,0.5};
    std::vector<double> delta_prev {0.5,0.5,0.5};

    int k{0};  //computing steps
    
    double                      x_previous;
    double                      y_previous;
    double                      z_previous;

    
    double                      offset_x;
    double                      offset_y;
    double                      offset_z;
    
    // ---------OUTPUT MSG-----------------------------------
    boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
    ros::ServiceClient client;
    mrs_msgs::Vec4 srv;
    //--------  COST FUNCTIONS ------------------------------
    double CostX(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,double offset_x)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(0) - x_prev.at<float>(0)),2) + std::pow((x.at<float>(0) - (master_pose.at<float>(0)+offset_x)),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(object_cov)*10e-4;  
        return resulting_cost;
    }
    double CostY(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,double offset_y)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(1) - x_prev.at<float>(1)),2) + std::pow((x.at<float>(1) - (master_pose.at<float>(1)+offset_y)),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(object_cov)*10e-4;  
        return resulting_cost;
    }
    double CostZ(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,double offset_z)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(2) - x_prev.at<float>(2)),2) + std::pow((x.at<float>(2) - (master_pose.at<float>(2)+offset_z)),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(object_cov)*10e-4;  
        return resulting_cost;
    }
    int sign(double x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }


    Formation(char** argv, double x_parameter,double y_parameter, double z_parameter)
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

        sub_1.subscribe(nh,sub_object_topic,1);
        sub_2.subscribe(nh,sub_pose_topic,1);
        sub_3.subscribe(nh,sub_yaw_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2,sub_3));
        sync->registerCallback(boost::bind(&Formation::callback,this,_1,_2,_3));
                
        client = nh.serviceClient<mrs_msgs::Vec4>(pub_pose_topic);
        
         // Gradient Descent Parameters
        x_previous = initial_state.x;
        y_previous = initial_state.y;
        z_previous = initial_state.z;

        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;

        tracker.push_back(tracker_vector);
        tracker.push_back(tracker_vector);

        ROS_INFO("Motion Controller Node Initialized Successfully"); 
    }
    void callback(const OdometryConstPtr& object,const OdometryConstPtr& pose, const EstimatedStateConstPtr& yaw)
    {
        ROS_INFO("Synchronized\n");

        if (object->pose.pose.position.x != '\0'){
            //------------MEASUREMENTS-----------------------------
            state = (cv::Mat_<float>(4,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,yaw->state[0]);
            state_cov = (cv::Mat_<double>(6,6)<<
                                                    pose->pose.covariance[0],
                                                    pose->pose.covariance[1],
                                                    pose->pose.covariance[2], 
                                                    pose->pose.covariance[3],
                                                    pose->pose.covariance[4],
                                                    pose->pose.covariance[5],
                                                    pose->pose.covariance[6],
                                                    pose->pose.covariance[7],
                                                    pose->pose.covariance[8],
                                                    pose->pose.covariance[9],
                                                    pose->pose.covariance[10],
                                                    pose->pose.covariance[11],
                                                    pose->pose.covariance[12], 
                                                    pose->pose.covariance[13],
                                                    pose->pose.covariance[14],
                                                    pose->pose.covariance[15],
                                                    pose->pose.covariance[16],
                                                    pose->pose.covariance[17],
                                                    pose->pose.covariance[18],
                                                    pose->pose.covariance[19],
                                                    pose->pose.covariance[20],
                                                    pose->pose.covariance[21],
                                                    pose->pose.covariance[22], 
                                                    pose->pose.covariance[23],
                                                    pose->pose.covariance[24],
                                                    pose->pose.covariance[25],
                                                    pose->pose.covariance[26],
                                                    pose->pose.covariance[27],
                                                    pose->pose.covariance[28],
                                                    pose->pose.covariance[29],
                                                    pose->pose.covariance[30],
                                                    pose->pose.covariance[31],
                                                    pose->pose.covariance[32], 
                                                    pose->pose.covariance[33],
                                                    pose->pose.covariance[34],
                                                    pose->pose.covariance[35]
                                                    );
            
            object_coord = (cv::Mat_<float>(3,1)<< object->pose.pose.position.x,object->pose.pose.position.y,object->pose.pose.position.z);
            object_cov   = (cv::Mat_<float>(6,6)<<   object->pose.covariance[0],
                                                    object->pose.covariance[1],
                                                    object->pose.covariance[2], 
                                                    object->pose.covariance[3],
                                                    object->pose.covariance[4],
                                                    object->pose.covariance[5],
                                                    object->pose.covariance[6],
                                                    object->pose.covariance[7],
                                                    object->pose.covariance[8],
                                                    object->pose.covariance[9],
                                                    object->pose.covariance[10],
                                                    object->pose.covariance[11],
                                                    object->pose.covariance[12], 
                                                    object->pose.covariance[13],
                                                    object->pose.covariance[14],
                                                    object->pose.covariance[15],
                                                    object->pose.covariance[16],
                                                    object->pose.covariance[17],
                                                    object->pose.covariance[18],
                                                    object->pose.covariance[19],
                                                    object->pose.covariance[20],
                                                    object->pose.covariance[21],
                                                    object->pose.covariance[22], 
                                                    object->pose.covariance[23],
                                                    object->pose.covariance[24],
                                                    object->pose.covariance[25],
                                                    object->pose.covariance[26],
                                                    object->pose.covariance[27],
                                                    object->pose.covariance[28],
                                                    object->pose.covariance[29],
                                                    object->pose.covariance[30],
                                                    object->pose.covariance[31],
                                                    object->pose.covariance[32], 
                                                    object->pose.covariance[33],
                                                    object->pose.covariance[34],
                                                    object->pose.covariance[35]
                                                    );
            //------------TRACKER----------------
            tracker.push_back(object_coord);
            count++;

            if (count == 2)
            {
                tracker.pop_back();
                tracker.pop_back();
            }
            if  (count > 22)
            {
                double sum_x{0},sum_y{0},sum_z{0};
                for (int i=0;i<22;i++)
                {
                    sum_x += tracker[i].at<float>(0);
                    sum_y += tracker[i].at<float>(1);
                    sum_z += tracker[i].at<float>(2);
                }
                master_pose = (cv::Mat_<float>(3,1) << sum_x/22,sum_y/22,sum_z/22);
                for (int i=0;i<10;i++)
                {
                    tracker.pop_back();
                    count--;
                }

            }
            
            w = (cv::Mat_<float>(3,1)<< state.at<float>(0),state.at<float>(1),state.at<float>(2)); 
            //------------RPROP----------------
            // goal-driven behaviour
            if (master_pose.empty() == false)
            {
                ROS_INFO_STREAM("master at"<<master_pose);
                // run optimization
                // costs
                cost_prev_x = CostX(w_prev,w_prev,master_pose,state_cov,object_cov,offset_x);
                cost_prev_y = CostY(w_prev,w_prev,master_pose,state_cov,object_cov,offset_y);
                cost_prev_z = CostZ(w_prev,w_prev,master_pose,state_cov,object_cov,offset_z);

                cost_cur_x = CostX(w,w_prev,master_pose,state_cov,object_cov,offset_x);
                cost_cur_y = CostY(w,w_prev,master_pose,state_cov,object_cov,offset_y);
                cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,object_cov,offset_z);

                cost_cur.push_back(cost_cur_x);
                cost_cur.push_back(cost_cur_y);
                cost_cur.push_back(cost_cur_z);

                cost_prev.push_back(cost_prev_x);
                cost_prev.push_back(cost_prev_y);
                cost_prev.push_back(cost_prev_z);

                cost_dif_x = (cost_cur_x - cost_prev_x);
                cost_dif_y = (cost_cur_y - cost_prev_y);
                cost_dif_z = (cost_cur_z - cost_prev_z);


                step_x = w.at<float>(0) - w_prev.at<float>(0);
                step_y = w.at<float>(1) - w_prev.at<float>(1);
                step_z = w.at<float>(2) - w_prev.at<float>(2);

                grad_prev.push_back(cost_dif_x/step_x);
                grad_prev.push_back(cost_dif_y/step_y);
                grad_prev.push_back(cost_dif_z/step_z);

                // computing longer when standing
                if ((std::abs(w.at<float>(0) - w_prev.at<float>(0))<0.2) || (std::abs(w.at<float>(1) - w_prev.at<float>(1))<0.2) || (std::abs(w.at<float>(2) - w_prev.at<float>(2))<0.2))
                {
                        k = 200;
                } else  k = 50;
                // -------------------------------------------------- 
                
            
                for(int j=0;j<k;j++)
                {
                    // Main RPROP loop
                    cost_cur_x = CostX(w,w_prev,master_pose,state_cov,object_cov,offset_x);
                    cost_cur_y = CostY(w,w_prev,master_pose,state_cov,object_cov,offset_y);
                    cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,object_cov,offset_z);

                    cost_cur[0] = cost_cur_x;
                    cost_cur[1] = cost_cur_y;
                    cost_cur[2] = cost_cur_z;

                    cost_dif_x = (cost_cur_x - cost_prev_x);
                    cost_dif_y = (cost_cur_y - cost_prev_y);
                    cost_dif_z = (cost_cur_z - cost_prev_z);

                    step_x = w.at<float>(0) - w_prev.at<float>(0);
                    step_y = w.at<float>(1) - w_prev.at<float>(1);
                    step_z = w.at<float>(2) - w_prev.at<float>(2);
                    
                    grad_cur[0] = cost_dif_x/step_x;
                    grad_cur[1] = cost_dif_y/step_y;
                    grad_cur[2] = cost_dif_z/step_z;

                    delta_prev = delta; 
                    for (int i = 0; i<3;i++)
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

                    cost_prev[0] = cost_prev_x;
                    cost_prev[1] = cost_prev_y;
                    cost_prev[2] = cost_prev_z;
                }
                // ----------------------------------
                srv.request.goal = boost::array<double, 4>{w.at<float>(0),w.at<float>(1),w.at<float>(2),std::round(atan2(master_pose.at<float>(1)-w.at<float>(1),master_pose.at<float>(0)-w.at<float>(0)))};

                if (client.call(srv))
                {
                    // ROS_INFO("Successfull calling service\n");
                    sleep(CONTROLLER_PERIOD);
                }
                else 
                {
                        ROS_ERROR("Could not publish\n");
                }
                // Optionally publish error
            }
            else
            {
                ROS_INFO_STREAM("[CALCULATING]:"<<count);
            }

            w_prev = (cv::Mat_<float>(3,1)<< state.at<float>(0),state.at<float>(1),state.at<float>(2)); 
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
    
    float offset_parameter_x;
    float offset_parameter_y;
    float offset_parameter_z;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;
 

    ROS_INFO_STREAM  ("Instanciating Motion Controller\n");
    ROS_INFO_STREAM  (argv[1]<<offset_parameter_x);
    std::string node_name = "";
    node_name += argv[4];
    node_name += "_formation_controller";
    
    ros::init(argc, argv, node_name);
    Formation fc(argv,offset_parameter_x,offset_parameter_y,offset_parameter_z);
    ros::spin();

    return 0;
}
