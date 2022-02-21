// Copyright [2021] [Timur Uzakov]

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
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


using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

int counter{0};


class SensFuse
{
public:

    
    ros::Publisher sf1_pub;
    ros::Publisher goal_pub;
    nav_msgs::Odometry goal_msg;
    nav_msgs::Odometry msg;

    message_filters::Subscriber<Odometry> sf1_sub;
    message_filters::Subscriber<Odometry> sf2_sub;
    message_filters::Subscriber<Odometry> pose_sub;
    message_filters::Subscriber<Odometry> object_sub;

    std::string sf1_topic    = "";
    std::string sf2_topic    = "";
    std::string object_topic = "";
    std::string pose_topic   = "";
    std::string goal_topic   = "";


    typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;



    // Cov matrix for message
    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32FC1, &msg_cov_array);

    // Cov matrix for message
    boost::array<double, 36UL> goal_msg_cov_array;
    cv::Mat goal_cov_matrix = cv::Mat(6,6,CV_32FC1, &goal_msg_cov_array);

    cv::Mat P_est, x_est, P, cur_state, R, C, C_transpose,S_inv, sf1_cov, sf2_cov;


    double prev_time_meas {0};

    SensFuse(ros::NodeHandle nh,char* name1, char* name2)
    {
        sf1_topic += "/";
        sf1_topic += name1;
        sf1_topic += "/sensor_fusion";

        std::cout<<sf1_topic<<std::endl;

        sf2_topic += "/";
        sf2_topic += name2;
        sf2_topic += "/sensor_fusion";

        std::cout<<sf2_topic<<std::endl;

        object_topic  += "/";
        object_topic  += name1;
        object_topic  += "/object/";

        pose_topic += "/";
        pose_topic += name1;
        pose_topic += "/odometry/odom_main/";

        goal_topic += "/";
        goal_topic += name1;
        goal_topic += "/goal/";

        sf1_sub.subscribe (nh,sf1_topic,1);
        sf2_sub.subscribe (nh,sf2_topic,1);
        pose_sub.subscribe  (nh,pose_topic,1);
        object_sub.subscribe (nh,object_topic,1);

        sf1_pub = nh.advertise<Odometry>(sf1_topic, 1);
        goal_pub = nh.advertise<Odometry>(goal_topic,1);


        sync.reset(new Sync(MySyncPolicy(10),sf1_sub,sf2_sub,object_sub,pose_sub));
        sync->registerCallback(boost::bind(&SensFuse::callback,this, _1,_2,_3,_4));


        //    init msg
        P_est = cv::Mat::eye(cv::Size(6,6),CV_32FC1);
        x_est = (cv::Mat_<float>(6,1)<<0,0,0,0,0,0);

        P     = cv::Mat::eye(cv::Size(6,6),CV_32FC1);
        sf1_cov     = cv::Mat::eye(cv::Size(6,6),CV_32FC1);
        sf2_cov     = cv::Mat::eye(cv::Size(6,6),CV_32FC1);
        // MSG
        cur_state = (cv::Mat_<float>(6,1)<<0,0,0,0,0,0);

        msg.pose.pose.position.x = cur_state.at<float>(0);
        msg.pose.pose.position.y = cur_state.at<float>(1);
        msg.pose.pose.position.z = cur_state.at<float>(2);
        msg.twist.twist.linear.x = cur_state.at<float>(3);
        msg.twist.twist.linear.y = cur_state.at<float>(4);
        msg.twist.twist.linear.z = cur_state.at<float>(5);

        cov_matrix = (cv::Mat_<float>(6,6) <<           1,0,0,0,0,0,
                                                        0,1,0,0,0,0,
                                                        0,0,1,0,0,0,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);

        msg.pose.covariance = msg_cov_array;
        msg.header.stamp = ros::Time::now();


        C = (cv::Mat_<float>(3,6) << 1,0,0,0,0,0,
                                     0,1,0,0,0,0,
                                     0,0,1,0,0,0);


        R = (cv::Mat_<float>(6,6) <<                    0.5,0,0,0,0,0,
                                                        0,0.5,0,0,0,0,
                                                        0,0,0.5,0,0,0,
                                                        0,0,0,0.5,0,0,
                                                        0,0,0,0,0.5,0,
                                                        0,0,0,0,0,0.5);
       R.convertTo(R, CV_32FC1);
       C.convertTo(C, CV_32FC1);
       P_est.convertTo(P_est, CV_32FC1);
       x_est.convertTo(x_est, CV_32FC1);


       cv::transpose(C,C_transpose);
       C_transpose.convertTo(C_transpose, CV_32FC1);

       x_est.convertTo(x_est, CV_32FC1);
       cur_state.convertTo(cur_state, CV_32FC1);

       P.convertTo(P, CV_32FC1);


       goal_msg.pose.pose.position.x = x_est.at<float>(0);
       goal_msg.pose.pose.position.y = x_est.at<float>(1);
       goal_msg.pose.pose.position.z = x_est.at<float>(2);
       goal_msg.twist.twist.linear.x = x_est.at<float>(3);
       goal_msg.twist.twist.linear.y = x_est.at<float>(4);
       goal_msg.twist.twist.linear.z = x_est.at<float>(5);
       goal_cov_matrix = P_est;
       goal_msg.pose.covariance = goal_msg_cov_array;
       goal_msg.header.stamp = ros::Time::now();

       ROS_INFO("All functions initialized");
    }

    cv::Mat ObjectCoordinateToWorld(cv::Mat object_position,double yaw_value,cv::Mat drone_position,cv::Mat offset_vector)
    {
        cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << 1280/2,720/2,0); //!TODO! Check image size

        cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) << 0.005,0,0,  0,-0.005,0,     0,0,1); //x same, y flip and rescale



        cv::Mat shifted_and_scaled  = scale_matrix * (object_position - shift_to_center);
        cv::Mat RotationMatrix      = (cv::Mat_<float>(3,3) << sin(yaw_value),0,cos(yaw_value),    -cos(yaw_value),0,sin(yaw_value) ,   0,1,0);

        cv::Mat rotated_vector      = RotationMatrix * shifted_and_scaled;

        // uncomment for debugging

        cv::Mat point = drone_position + offset_vector + rotated_vector;
        std::cout<<drone_position<<'\n';
        return point;
    }



    void callback(const OdometryConstPtr sf1, const OdometryConstPtr sf2,const OdometryConstPtr object, const OdometryConstPtr pose)
    {
        ROS_INFO_STREAM("Synchronized");
        // Conversion of state vectors and their covariances into CV::Mat format
        cv::Mat sf1_state = (cv::Mat_<float>(6,1) << sf1->pose.pose.position.x,sf1->pose.pose.position.y,
                                                     sf1->pose.pose.position.z, sf1->twist.twist.linear.x,
                                                     sf1->twist.twist.linear.y,sf1->twist.twist.linear.z);
        

        sf1_cov   = (cv::Mat_<double>(6,6) <<
                                                sf1->pose.covariance[0],
                                                sf1->pose.covariance[1],
                                                sf1->pose.covariance[2],
                                                sf1->pose.covariance[3],
                                                sf1->pose.covariance[4],
                                                sf1->pose.covariance[5],
                                                sf1->pose.covariance[6],
                                                sf1->pose.covariance[7],
                                                sf1->pose.covariance[8],
                                                sf1->pose.covariance[9],
                                                sf1->pose.covariance[10],
                                                sf1->pose.covariance[11],
                                                sf1->pose.covariance[12],
                                                sf1->pose.covariance[13],
                                                sf1->pose.covariance[14],
                                                sf1->pose.covariance[15],
                                                sf1->pose.covariance[16],
                                                sf1->pose.covariance[17],
                                                sf1->pose.covariance[18],
                                                sf1->pose.covariance[19],
                                                sf1->pose.covariance[20],
                                                sf1->pose.covariance[21],
                                                sf1->pose.covariance[22],
                                                sf1->pose.covariance[23],
                                                sf1->pose.covariance[24],
                                                sf1->pose.covariance[25],
                                                sf1->pose.covariance[26],
                                                sf1->pose.covariance[27],
                                                sf1->pose.covariance[28],
                                                sf1->pose.covariance[29],
                                                sf1->pose.covariance[30],
                                                sf1->pose.covariance[31],
                                                sf1->pose.covariance[32],
                                                sf1->pose.covariance[33],
                                                sf1->pose.covariance[34],
                                                sf1->pose.covariance[35]
                                                );
        cv::Mat sf2_state = (cv::Mat_<float>(6,1) << sf2->pose.pose.position.x,sf2->pose.pose.position.y,
                                                     sf2->pose.pose.position.z, sf2->twist.twist.linear.x,
                                                     sf2->twist.twist.linear.y,sf2->twist.twist.linear.z);

        sf2_cov   = (cv::Mat_<double>(6,6) <<
                                                sf2->pose.covariance[0],
                                                sf2->pose.covariance[1],
                                                sf2->pose.covariance[2],
                                                sf2->pose.covariance[3],
                                                sf2->pose.covariance[4],
                                                sf2->pose.covariance[5],
                                                sf2->pose.covariance[6],
                                                sf2->pose.covariance[7],
                                                sf2->pose.covariance[8],
                                                sf2->pose.covariance[9],
                                                sf2->pose.covariance[10],
                                                sf2->pose.covariance[11],
                                                sf2->pose.covariance[12],
                                                sf2->pose.covariance[13],
                                                sf2->pose.covariance[14],
                                                sf2->pose.covariance[15],
                                                sf2->pose.covariance[16],
                                                sf2->pose.covariance[17],
                                                sf2->pose.covariance[18],
                                                sf2->pose.covariance[19],
                                                sf2->pose.covariance[20],
                                                sf2->pose.covariance[21],
                                                sf2->pose.covariance[22],
                                                sf2->pose.covariance[23],
                                                sf2->pose.covariance[24],
                                                sf2->pose.covariance[25],
                                                sf2->pose.covariance[26],
                                                sf2->pose.covariance[27],
                                                sf2->pose.covariance[28],
                                                sf2->pose.covariance[29],
                                                sf2->pose.covariance[30],
                                                sf2->pose.covariance[31],
                                                sf2->pose.covariance[32],
                                                sf2->pose.covariance[33],
                                                sf2->pose.covariance[34],
                                                sf2->pose.covariance[35]
                                                );



        // Measurement

        cv::Mat state  = (cv::Mat_<float>(3,1) << pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z );
        
        
        cv::Mat offset = (cv::Mat_<float>(3,1) << (0.2*cos(pose->pose.pose.orientation.z)),(0.2*sin(pose->pose.pose.orientation.z)),0);
        cv::Mat object_coord = (cv::Mat_<float>(3,1)<< object->pose.pose.position.x, object->pose.pose.position.y,object->pose.pose.position.z);

        cv::Mat meas_state = ObjectCoordinateToWorld(object_coord,pose->pose.pose.orientation.z,state,offset);
        cv::Mat meas_cov   = (cv::Mat_<double>(6,6)<<
                                                object->pose.covariance[0],
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
                                                );;

        // KALMAN
        // Correction

        meas_cov.convertTo(meas_cov, CV_32FC1);
        meas_state.convertTo(meas_state, CV_32FC1);
        
        
        sf1_state.convertTo(sf1_state, CV_32FC1);
        sf2_state.convertTo(sf2_state, CV_32FC1);

        sf1_cov.convertTo(sf1_cov, CV_32FC1);
        sf2_cov.convertTo(sf2_cov, CV_32FC1);


        

        cv::Mat S = C * P_est * C_transpose + C * meas_cov * C_transpose;
        
        cv::invert(S,S_inv,cv::DECOMP_SVD);

        S.convertTo(S, CV_32FC1);
        S_inv.convertTo(S_inv,CV_32FC1);

        cv::Mat K = P_est * C_transpose * S_inv; 
        

        K.convertTo(K, CV_32FC1);

        cv::Mat innovation = meas_state - C * x_est;

        innovation.convertTo(innovation, CV_32FC1);
        // sf1 output
        
        

        cur_state = x_est + K * innovation;


        P = cv::Mat::eye(cv::Size(6,6),CV_32FC1) - K * C * P_est;
        
        

        // publishing cur state
        // MSG

        
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = cur_state.at<float>(0);
        msg.pose.pose.position.y = cur_state.at<float>(1);
        msg.pose.pose.position.z = cur_state.at<float>(2);
        msg.twist.twist.linear.x = cur_state.at<float>(3);
        msg.twist.twist.linear.y = cur_state.at<float>(4);
        msg.twist.twist.linear.z = cur_state.at<float>(5);
        cov_matrix = P;
        msg.pose.covariance = msg_cov_array;
        sf1_pub.publish(msg);

        

        double time_meas = object->header.stamp.sec + pose->header.stamp.nsec/1000000000;
        double time_dif = time_meas - prev_time_meas;

        cv::Mat A = (cv::Mat_<float>(6,6) <<            1,0,0,time_dif,0,0,
                                                        0,1,0,0,time_dif,0,
                                                        0,0,1,0,0,time_dif,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        cv::Mat A_transposed = (cv::Mat_<float>(6,6) << 1,0,0,0,0,0,
                                                        0,1,0,0,0,0,
                                                        0,0,1,0,0,0,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        cv::transpose(A, A_transposed);
        A.convertTo(A, CV_32FC1);
        A_transposed.convertTo(A_transposed, CV_32FC1);


       // Prediction
        x_est = A * ((sf1_state+sf2_state)/2.0);
        // ROS_INFO_STREAM("Estimated state: x "<<x_est.at<float>(0)<<" y "<<x_est.at<float>(1)<<" z "<<x_est.at<float>(2));

        P_est = (A* ((sf1_cov + sf2_cov)/ 2.0) * A_transposed) + R;
// Porblem here:
        ROS_INFO_STREAM("A" << A <<std::endl;);
        ROS_INFO_STREAM("R" << R <<std::endl;);
        ROS_INFO_STREAM("sum cov" <<((sf1_cov + sf2_cov)/ 2.0)<<std::endl;);


        

        goal_msg.pose.pose.position.x = x_est.at<float>(0);
        goal_msg.pose.pose.position.y = x_est.at<float>(1);
        goal_msg.pose.pose.position.z = x_est.at<float>(2);
        goal_msg.twist.twist.linear.x = x_est.at<float>(3);
        goal_msg.twist.twist.linear.y = x_est.at<float>(4);
        goal_msg.twist.twist.linear.z = x_est.at<float>(5);
        goal_cov_matrix = P_est;
        goal_msg.pose.covariance = goal_msg_cov_array;
        goal_msg.header.stamp = ros::Time::now();

        goal_pub.publish(goal_msg);

        prev_time_meas = time_meas;


        ros::Rate rate(100);
        rate.sleep();
    }
};

int main(int argc, char** argv)
{
    ROS_INFO("Sensor Fusion node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_sensor_fusion";

    ros::init(argc, argv, node_name);
    std::cout<<node_name<<std::endl;
    ros::NodeHandle nh;
    SensFuse sf(nh,argv[1], argv[2]);
    
    while(!ros::isShuttingDown())
    {
        sf.msg.header.stamp = ros::Time::now();
        sf.sf1_pub.publish(sf.msg);
        counter++;
        ros::spinOnce();
    }
    return 0;
}
