// Copyright [2021] [Timur Uzakov]
// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
// include ros library
#include <ros/ros.h>
// Time Sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//  Messages
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>

// Math
#include <math.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class MotionController 
{
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<PointCloud>     sub_1;
    message_filters::Subscriber<Image>          sub_2;
    message_filters::Subscriber<Odometry>       sub_3;
    typedef sync_policies::ApproximateTime<PointCloud,Image,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // image_transport::Publisher  pub;
    ros::Publisher              pub_go_to;
    int count = 0;
    std::string                 sub_point_cloud_topic = "";
    std::string                 sub_image_topic       = "";
    std::string                 sub_goal_topic        = "";

    std::string                 pub_go_to_point_topic = "";
    
    // sensor_msgs::ImagePtr       msg_output;
    PointStamped                msg_go_to;
    
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
    // ------------------------------------------ 

    double CostFunction(double pose_x, double x, double offset_robot_pose, std::vector<float> obstacles)
    {   
    //  Quadratic optimization function
    //  Offset determines a robot's position
        double obstacle_cost {0};
        for(auto& obstacle : obstacles)
        {
            obstacle_cost += 1/(pow((x-obstacle),2)+0.1);
        }
        // uncomment for debugging
        // std::cout<<"obstacle cost: "<<0.1*obstacle_cost<<'\n';
        return pow((x-(pose_x + offset_robot_pose)),2) + 0.1*obstacle_cost;
    }
    double GradientFunction(double pose_x, double x, double offset_robot_pose,std::vector<float> obstacles)
    {
    //  Gradient of the cost function
    //  Offset determines a robot's position
        double obstacle_cost {0};
        double obstacle_gradient {0};
        for(auto& obstacle : obstacles)
        {
            obstacle_gradient += -2*pow((x-obstacle)+0.1,-3);
        }
        return 2*(x-(pose_x + offset_robot_pose))+ obstacle_gradient;
    }

    double CalculateUpdate(double pose_x, double previous,double offset_robot_pose,std::vector<float> obstacles)
    {
    //  Gradient descent update
    //  Offset determines a robot's position
    double alpha {0.1}; //step parameter
    double gradient;
    double updated;
    
    gradient     = GradientFunction(pose_x, previous,offset_robot_pose,obstacles);
    updated    = previous - gradient*alpha;
    return  updated;
    }

    double FindGoToPoint(double cost, double pose_x, double previous, double offset_robot_pose,std::vector<float> obstacles)
    {
        // Main loop for finding a go_to point
        // While cost function is greated than threshold, the state will be updating for n steps
        // In case cost function is lower than threshold, the state is preserved
        // pose_x is the coordinate of goal
        double updated {0};
        int    number_of_steps {100};
        for (int j=0;j<number_of_steps;j++)
        {
                updated       = CalculateUpdate(pose_x,previous,offset_robot_pose, obstacles);
                
                // uncomment for debugging purposes
                // cost          = CostFunction(pose_x, updated,offset); 
                // ROS_INFO_STREAM("Current Cost:" <<cost<<'\n');      
                previous      = updated;
        }
        return updated;

    }
    cv::Mat HumanCoordinateToWorld(cv::Mat image,cv::Mat object_position,double yaw_value,cv::Mat drone_position,cv::Mat offset_vector)
    {
        cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << image.size[1]/2,image.size[0]/2,0);
        cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) << 0.005,0,0,  0,-0.005,0,     0,0,1); //x same, y flip and rescale
        // uncomment for debugging 
        // std::cout<<shift_to_center<<'\n';
        cv::Mat shifted_and_scaled  = scale_matrix*(object_position - shift_to_center);
        cv::Mat R                   = (cv::Mat_<float>(3,3) << sin(yaw_value),0,cos(yaw_value),    -cos(yaw_value),0,sin(yaw_value) ,   0,1,0);
        cv::Mat rotated_vector      = R*shifted_and_scaled;

        cv::Mat point = drone_position + offset_vector + rotated_vector;
        
        return point;
    }

    MotionController(double x_parameter,double y_parameter, double z_parameter, char * name)
    {   
        sub_point_cloud_topic  += "/";
        sub_point_cloud_topic += name;
        sub_point_cloud_topic  += "/ORB/points";

        sub_image_topic += "/";
        sub_image_topic += name;
        sub_image_topic += "/camera/image";

        sub_goal_topic += "/";
        sub_goal_topic += name;
        sub_goal_topic += "/goal";

        pub_go_to_point_topic+= "/";
        pub_go_to_point_topic+= name;
        pub_go_to_point_topic+= "/camera/go_to";

        sub_1.subscribe(nh,sub_point_cloud_topic,1);
        sub_2.subscribe(nh,sub_image_topic,1);
        sub_3.subscribe(nh,sub_goal_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2,sub_3));
        sync->registerCallback(boost::bind(&MotionController::callback,this,_1,_2,_3));
        image_transport::ImageTransport it(nh);
        
         // Gradient Descent Parameters
        x_previous = initial_state.x;
        y_previous = initial_state.y;
        z_previous = initial_state.z;

        // Taking parameters to set robot position
        offset_x = x_parameter;
        offset_y = y_parameter;
        offset_z = z_parameter;

        // pub = it.advertise(pub_motion_topic, 1);
        pub_go_to = nh.advertise<PointStamped> (pub_go_to_point_topic,1);

        ROS_INFO("Motion Controller Node Initialized Successfully"); 
    }
    void callback(const sensor_msgs::PointCloudConstPtr obstacles,const sensor_msgs::ImageConstPtr& msg, const OdometryConstPtr goal)
    {
        // uncomment for debugging
        // std::cout<<offset_x<<'\n';
        // std::cout<<offset_y<<'\n';
        // std::cout<<offset_z<<'\n';

        ROS_INFO_STREAM("[ Messages synchronized ]");
        // Receiving and converting image to CV Mat object
        std_msgs::Header    msg_header = msg->header;
        std::string         frame_id = msg_header.frame_id;
        // ROS_INFO_STREAM("[Image from: " << frame_id<<" ]");
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        
        // ----------------------------obstacles----------------
        std::vector<float> obstacles_x;
        std::vector<float> obstacles_y;
        std::vector<float> obstacles_z;
        
        for (int i = 0; i<obstacles->points.size();i++)
        {   
            cv::Mat obstacle_mat = (cv::Mat_<float>(3,1) << obstacles->points[i].x, obstacles->points[i].y, obstacles->points[i].z);
            
            // obstacles_x.push_back(goal->pose.pose.position.x);
            // obstacles_y.push_back(goal->pose.pose.position.y);
            // obstacles_z.push_back(goal->pose.pose.position.z);
            //!TODO: Configure depth position in ORB Detector
        }

        // -----------------------------------------------------
        
        
        ROS_INFO_STREAM("[Goal world position]");
        ROS_INFO_STREAM("["<<goal->pose.pose.position.x<<" | "<<goal->pose.pose.position.y<<" | "<<goal->pose.pose.position.z<<"]");
        // -------------------------------------------------

        // Gradient Descent parameters    
        double x_updated, y_updated, z_updated;
        double current_cost_x, current_cost_y, current_cost_z;
       
        // Calculation of cost function values
        current_cost_x = CostFunction(goal->pose.pose.position.x, x_previous,offset_x, obstacles_x);
        current_cost_y = CostFunction(goal->pose.pose.position.y, y_previous,offset_y, obstacles_y);
        current_cost_z = CostFunction(goal->pose.pose.position.z, z_previous,offset_z, obstacles_z);
        
        // Determining the optimal state
        x_updated = FindGoToPoint(current_cost_x, goal->pose.pose.position.x, x_previous,offset_x, obstacles_x);
        y_updated = FindGoToPoint(current_cost_y, goal->pose.pose.position.y, y_previous,offset_y, obstacles_y);
        z_updated = FindGoToPoint(current_cost_z, goal->pose.pose.position.z, z_previous,offset_z, obstacles_z);

        ROS_INFO_STREAM("[GoTo Destination Position Found]");        
        ROS_INFO_STREAM("["<<x_updated<<" | "<<y_updated<<" | "<<z_updated<<"]");
        
        //!TODO: yaw optimization towards goal


        // publishing image
        // msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        // msg_output->header.frame_id = std::to_string(count);
        // pub.publish(msg_output);

        // publishing go_to point

        msg_go_to.header.frame_id   = frame_id;
        msg_go_to.header.stamp = ros::Time::now();
        msg_go_to.point.x = x_updated;
        msg_go_to.point.y = y_updated;
        msg_go_to.point.z = z_updated;

        pub_go_to.publish(msg_go_to);

        // update
        count++;        
        x_previous = x_updated;
        y_previous = y_updated;
        z_previous = z_updated;

    }
};




int main(int argc, char** argv)

{
     // Check if video source has been passed as a parameter ---------
    if(argv[1] == NULL) 
    {
        std::cerr<<"Please, enter the drone position, in the following form: x y z"<<'\n';
    return 1; 
    }
    std::istringstream              source_cmd_x(argv[1]);
    std::istringstream              source_cmd_y(argv[2]);
    std::istringstream              source_cmd_z(argv[3]);
    
    double offset_parameter_x;
    double offset_parameter_y;
    double offset_parameter_z;

    if(!(source_cmd_x>>offset_parameter_x)) return 1;
    if(!(source_cmd_y>>offset_parameter_y)) return 1;
    if(!(source_cmd_z>>offset_parameter_z)) return 1;


    ROS_INFO_STREAM  ("Instanciating Motion Controller\n");
    ros::init        (argc, argv, "roscpp_open_cv");
    ros::NodeHandle  nh;

    MotionController mc(offset_parameter_x,offset_parameter_y,offset_parameter_z, argv[4]);
    ros::spin();
    return 0;
}
