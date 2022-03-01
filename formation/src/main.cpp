// Copyright [2021] [Timur Uzakov]
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec4.h>
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

class Formation
{
private:
    ros::NodeHandle nh;   
    message_filters::Subscriber<Odometry>     sub_1;
    message_filters::Subscriber<Odometry>     sub_2;

    typedef sync_policies::ApproximateTime<Odometry, Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


    // image_transport::Publisher  pub;
    int count = 0;

    std::string                 sub_pose_topic   = "";
    std::string                 sub_object_topic = "";
    std::string                 pub_pose_topic   = "";
    
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);
    cv::Scalar                  goal_color = cv::Scalar(50,255,255);
    
    cv::Point3f                 initial_state    = cv::Point3f(0,0,1);
    
public:

    double                      x_previous;
    double                      y_previous;
    double                      z_previous;

    // -------- ROBOT POSITION PARAMETERS -------
    // Set robot's relative to goal position
    // here:
    double offset_x {0};
    double offset_y {0};
    double offset_z {0};
                
    boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
    cv::Mat pose_cov;
    cv::Mat object_cov;

    ros::ServiceClient client;
    mrs_msgs::Vec4 srv;

    double CostFunction(double pose_x, double x, double offset_robot_pose)
    {   
        //  Quadratic optimization function
        //  Offset determines a robot's position
        return pow((x-(pose_x + offset_robot_pose)),2);
    }
    double GradientFunction(double pose_x, double x, double offset_robot_pose)
    {
        //  Gradient of the cost function
        //  Offset determines a robot's position
        //  Covariance is considered to be const
        return 2*(x-(pose_x + offset_robot_pose) +  0   );
    }
    
    double CalculateUpdate(double pose_x, double previous,double offset_robot_pose)
    {
    //  Gradient descent update
    //  Offset determines a robot's position
    double alpha {0.1}; //step parameter
    double gradient;
    double updated;
    
    gradient   = GradientFunction(pose_x, previous,offset_robot_pose);
    updated    = previous - gradient*alpha;
    return  updated;
    }

    double FindGoToPoint(double cost, double pose_x, double previous, double offset_robot_pose)
    {
        // Main loop for finding a go_to point
        // While cost function is greated than threshold, the state will be updating for n steps
        // In case cost function is lower than threshold, the state is preserved
        // pose_x is the coordinate of goal
        double alpha {0.1}; //step parameter
        double gradient;
        double updated {0};
        double cost_prev {1000000000000};
        int    number_of_steps {100};
        while(cost>0.1)
        {
            gradient   = GradientFunction(pose_x, previous,offset_robot_pose);
            updated    = previous - gradient*alpha;
            cost       = CostFunction(pose_x, updated,offset_robot_pose);
            if (cost>cost_prev)
            {
                updated    = previous + gradient*alpha;
            }
            else {
                updated = previous - gradient*alpha;
            }
            ROS_INFO_STREAM("Current Cost:" <<cost<<'\n');      
            previous      = updated;
            cost_prev = cost;
        }
        return updated;

    }


    Formation(char* name, double x_parameter,double y_parameter, double z_parameter)
    {
        sub_object_topic+="/";
        sub_object_topic+=name;
        sub_object_topic+="/goal/";

        sub_pose_topic  +="/";
        sub_pose_topic  +=name;
        sub_pose_topic  +="/odometry/odom_main/";

        sub_1.subscribe(nh,sub_object_topic,1);
        sub_2.subscribe(nh,sub_pose_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&Formation::callback,this,_1,_2));
        
         // Gradient Descent Parameters
        x_previous = initial_state.x;
        y_previous = initial_state.y;
        z_previous = initial_state.z;

        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;

        pub_pose_topic+="/";
        pub_pose_topic+=name;
        pub_pose_topic+="/control_manager/goto/";

        client = nh.serviceClient<mrs_msgs::Vec4>(pub_pose_topic);

        ROS_INFO("Motion Controller Node Initialized Successfully"); 
    }
    void callback(const OdometryConstPtr& object,const OdometryConstPtr& pose)
    {
        ROS_INFO_STREAM("Synchronized\n");      
        // uncomment for debugging
        // std::cout<<offset_x<<'\n';
        // std::cout<<offset_y<<'\n';
        // std::cout<<offset_z<<'\n';
        cv::Mat drone_position  = (cv::Mat_<float>(3,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z);
        cv::Mat object_position = (cv::Mat_<float>(3,1) << object->pose.pose.position.x,object->pose.pose.position.y,object->pose.pose.position.z);
        cv::Mat offset_vector   = (cv::Mat_<float>(3,1) << (0.2*cos(pose->pose.pose.orientation.z)),(0.2*sin(pose->pose.pose.orientation.z)),0);

        // ROS_INFO_STREAM("[Goal world position]");
        // ROS_INFO_STREAM("["<<object_position.at<float>(0)<<" | "<<object_position.at<float>(1)<<" | "<<object_position.at<float>(2)<<"]");

         // Gradient Descent parameters    
        double x_updated, y_updated, z_updated;
        double current_cost_x, current_cost_y, current_cost_z;

        // Calculation of cost function values
        current_cost_x = CostFunction(object_position.at<float>(0), x_previous,offset_x);
        current_cost_y = CostFunction(object_position.at<float>(1), y_previous,offset_y);
        current_cost_z = CostFunction(object_position.at<float>(2), z_previous,offset_z);
        
        // Determining the optimal state
        x_updated = FindGoToPoint(current_cost_x, object_position.at<float>(0), x_previous,offset_x);
        y_updated = FindGoToPoint(current_cost_y, object_position.at<float>(1), y_previous,offset_y);
        z_updated = FindGoToPoint(current_cost_z, object_position.at<float>(2), z_previous,offset_z);

        // ROS_INFO_STREAM("[GoTo Destination Position Found]");        
        // ROS_INFO_STREAM("["<<x_updated<<" | "<<y_updated<<" | "<<z_updated<<"]");

        ROS_INFO_STREAM(object_position.at<float>(0));
        ROS_INFO_STREAM(x_updated);


        srv.request.goal = boost::array<double, 4>{x_updated,y_updated,z_updated, std::round(atan2(object_position.at<float>(1)-drone_position.at<float>(1),object_position.at<float>(0)-drone_position.at<float>(0)))};
    
        if (client.call(srv))
        {
            // ROS_INFO("Successfull calling service\n");
            sleep(1);
        }
        else 
        {
            ROS_ERROR("Could not publish\n");
        }

        // update
        count++;        
        x_previous = x_updated;
        y_previous = y_updated;
        z_previous = z_updated;
    }

};



int main(int argc, char** argv)
{
    
    if(argv[1] == NULL) 
    {
        std::cerr<<"Please, enter the drone position around the goal, in the following form: UAV_NAME x y z"<<'\n';
        return 1; 
    }
    std::istringstream              source_cmd_x(argv[2]);
    std::istringstream              source_cmd_y(argv[3]);
    std::istringstream              source_cmd_z(argv[4]);
    
    float offset_parameter_x;
    float offset_parameter_y;
    float offset_parameter_z;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;
 

    ROS_INFO_STREAM  ("Instanciating Motion Controller\n");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_formation_controller";
    
    ros::init(argc, argv, node_name);
    Formation fc(argv[1],offset_parameter_x,offset_parameter_y,offset_parameter_z);
    ROS_INFO("Formation node initialized");
    ros::spin();

    return 0;
}
