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

#define CONTROLLER_PERIOD 8
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
    message_filters::Subscriber<Odometry>       sub_1;
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

    float resulting_cost_x {0};
    float resulting_cost_y {0};
    float resulting_cost_z {0};

    float angle = 0.0;
    float radius = RADIUS;
    
    float goal_x {0.0};
    float goal_y {0.0};
    float goal_z {2.5};
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

        pub_pose_topic+="/";
        pub_pose_topic+=argv[4];
        pub_pose_topic+="/control_manager/goto";
        ROS_INFO_STREAM(pub_pose_topic);

        sub_1.subscribe(nh,sub_pose_topic,1);
        sub_2.subscribe(nh,sub_yaw_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&Formation::callback,this,_1,_2));
                
        client = nh.serviceClient<mrs_msgs::Vec4>(pub_pose_topic);

        // Taking parameters to set robot position
        center_x = x_parameter;
        center_y = y_parameter;
        center_z = z_parameter;

        ROS_INFO("Leader Controller Node Initialized Successfully"); 
    }
    void callback(const OdometryConstPtr& pose, const EstimatedStateConstPtr& yaw)
    {
        ROS_INFO("Synchronized\n");
        angle += M_PI/16;

        if (angle >= 2*M_PI)
        {
            angle = 0;
        }
        goal_x = center_x + radius * cos(angle);
        goal_y = center_y + radius * sin(angle);
        ROS_INFO_STREAM("angle: "<<angle<<" radius: "<<radius<<'\n');
        ROS_INFO_STREAM("goal x: "<<goal_x<<" goal y: "<<goal_y<<'\n');

        srv.request.goal = boost::array<float, 4>{goal_x,goal_y,goal_z,0.0};
      
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
