// Copyright [2022] [Timur Uzakov]
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <queue>

// ---------------------MACROS--------------------------------------
#define COV_RESOLUTION 2
#define COV_DISTANCE_BETWEEN_POINTS 0.05
// #define ELLIPSE_SCALE  0.0645 
#define ELLIPSE_SCALE  0.03225/2

#define PRINT_OUT 0
#define SCALE95 2.447652
#define SIZE_OF_OBJECT 0.5
// -----------------------------------------------------------------
using namespace geometry_msgs;
using namespace message_filters;
using namespace mrs_msgs;
using namespace nav_msgs;
using namespace sensor_msgs;

class Visualiser
{

public:

// ---------------------PUB and SUB---------------------------------
    // ros::Publisher pub_cubes;
    // ros::Publisher pub_cubes_cov;
    // ros::Publisher pub_center;
    // ros::Publisher pub_center_cov;
    ros::Publisher pub_markers;


    message_filters::Subscriber<PoseWithCovarianceArrayStamped> obj_sub;
    message_filters::Subscriber<PoseWithCovarianceArrayStamped> obj_secondary_sub;
    // message_filters::Subscriber<PoseWithCovarianceArrayStamped> obj_third_sub;
// -----------------------------------------------------------------

    std::string sub_obj_topic           = "";
    std::string sub_obj_topic_secondary = "";
    std::string sub_obj_topic_third     = "";
    std::string sub_pose_topic          = "";
    std::string sub_yaw_topic           = "";

    std::string pub_cubes_topic               = "";
    std::string pub_cubes_cov_topic           = "";
    std::string pub_center_topic              = "";
    std::string pub_center_cov_topic          = "";
    std::string pub_markers_topic             = "";

    //---Kalman Filter Parameters---->>----

    cv::KalmanFilter KF = cv::KalmanFilter(6,6,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(6,1);

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    cv::Point3f center3D;

    cv::Mat main_cov,secondary_cov, third_cov;
    // ---<< Kalman Filter Parameters ----
    //---------------------------------------------------------
    u_int64_t count {0};
    //-----------------------------------------------------------------------------------------------------------------
    typedef sync_policies::ApproximateTime<PoseWithCovarianceArrayStamped,PoseWithCovarianceArrayStamped> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    float l1,l2,l3;
    float R1,R2,R3;

    Visualiser(ros::NodeHandle nh,const char* &name_main, const char* &name_secondary)
    {   

    // ---------------------INITIALIZATION--------------------------------------
        //subscribers

        sub_obj_topic  += "/";
        sub_obj_topic  += name_main;
        sub_obj_topic  += "/points/";

        sub_obj_topic_secondary  += "/";
        sub_obj_topic_secondary  += name_secondary;
        sub_obj_topic_secondary  += "/points/";

        // sub_obj_topic_third  += "/";
        // sub_obj_topic_third  += name_third;
        // sub_obj_topic_third  += "/points/";

        sub_yaw_topic  += "/";
        sub_yaw_topic  += name_main;
        sub_yaw_topic  += "/odometry/heading_state_out/";

        sub_pose_topic += "/";
        sub_pose_topic += name_main;
        sub_pose_topic += "/odometry/odom_main/";

        obj_sub.subscribe(nh,sub_obj_topic,1);
        obj_secondary_sub.subscribe (nh,sub_obj_topic_secondary,1);
        
        // obj_third_sub.subscribe (nh,sub_obj_topic_third,1);


        // publishers
        // pub_cubes_topic += "/visualiser";
        // pub_cubes_topic += "/cubes/";

        // pub_cubes_cov_topic += "/visualiser";
        // pub_cubes_cov_topic += "/cubes_cov/";

        // pub_center_topic += "/visualiser";
        // pub_center_topic += "/center/";

        // pub_center_cov_topic += "/visualiser";
        // pub_center_cov_topic += "/center_cov/";

        pub_markers_topic += "/visualiser";
        pub_markers_topic += "/markers/";



        // pub_cubes       = nh.advertise<visualization_msgs::Marker>(pub_cubes_topic,1);        
        // pub_cubes_cov   = nh.advertise<visualization_msgs::Marker>(pub_cubes_cov_topic,1);        
        // pub_center      = nh.advertise<visualization_msgs::Marker>(pub_center_topic,1);        
        // pub_center_cov  = nh.advertise<visualization_msgs::Marker>(pub_center_cov_topic,1);        
        
        pub_markers     = nh.advertise<visualization_msgs::MarkerArray>(pub_markers_topic,1);        


        sync.reset(new Sync(MySyncPolicy(10),obj_sub,obj_secondary_sub));
        sync->registerCallback(boost::bind(&Visualiser::callback_three,this, _1,_2));
        

        //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(6,6) <<  1,0,0,1,0,0,
                                                        0,1,0,0,1,0,
                                                        0,0,1,0,0,1,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        // measurement.setTo(cv::Scalar(0));
        

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(10)); //Q
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(100)); //R
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));


        ROS_INFO("[Visualiser] All functions initialized");
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

    int sign(float x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

    float getAverage(std::vector<float> v)
    {
        if (v.empty()) {
            return 0;
        }else
        {
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        }
    }
    visualization_msgs::Marker drawCovariance(visualization_msgs::Marker cov_marker, PoseWithCovarianceIdentified point){
        // ---- covariance visualization ------

            //create
            geometry_msgs::Point p;
            float sigma_x = point.covariance[9];
            float sigma_y = point.covariance[10];
            float sigma_z = point.covariance[11];

            double K1 = (double) ELLIPSE_SCALE*sigma_x;
            double K2 = (double) ELLIPSE_SCALE*sigma_y;
            double K3 = (double) ELLIPSE_SCALE*sigma_z;
        
            cv::Mat eig_vec_one = (cv::Mat_<float>(3,1)<<(float)point.covariance[0],(float)point.covariance[1],(float)point.covariance[2]);
            cv::Mat eig_vec_two = (cv::Mat_<float>(3,1)<<(float)point.covariance[3],(float)point.covariance[4],(float)point.covariance[5]);
            cv::Mat eig_vec_three = (cv::Mat_<float>(3,1)<<(float)point.covariance[6],(float)point.covariance[7],(float)point.covariance[8]);
             
            for(float i=-COV_RESOLUTION;i<COV_RESOLUTION;i+=COV_DISTANCE_BETWEEN_POINTS)
            {
                for(float j =-COV_RESOLUTION;j<COV_RESOLUTION;j+=COV_DISTANCE_BETWEEN_POINTS)
                {
                    for(float k=-COV_RESOLUTION;k<COV_RESOLUTION;k+=COV_DISTANCE_BETWEEN_POINTS)
                    {
                        cv::Mat sample_point = (cv::Mat_<float>(3,1)<<i,j,k);
                        double first_distance  = eig_vec_one.dot(sample_point);
                        double second_distance = eig_vec_two.dot(sample_point);
                        double third_distance  = eig_vec_three.dot(sample_point);

                        if(std::abs((std::pow((first_distance/K1),2) + std::pow((second_distance/K2),2)+std::pow((third_distance/K3),2))-1.0)<=0.01){
                           p.x = point.pose.position.x+i;
                           p.y = point.pose.position.y+j;
                           p.z = point.pose.position.z+k;

                           cov_marker.points.push_back(p);
                        }
                    }
                }
            }
        return cov_marker;
            // ---------------------------------------
    }
    
    void callback_three(PoseWithCovarianceArrayStampedConstPtr obj,PoseWithCovarianceArrayStampedConstPtr obj_secondary)
    {

        if (PRINT_OUT == 1)
            ROS_INFO_STREAM("Synchronized");
        cv::Point3f predictPt = PredictUsingKalmanFilter();
        //------------MEASUREMENTS------------------------ 

        visualization_msgs::MarkerArray marker_array;

        // ---- Marker message with centroids ----
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "marker_cube_list";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        // marker.pose.position.x = 0;
        // marker.pose.position.y = 0;
        // marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0; // 1.0
        marker.scale.x = SIZE_OF_OBJECT;
        marker.scale.y = SIZE_OF_OBJECT;
        marker.scale.z = SIZE_OF_OBJECT;
        marker.color.r = 0.54;
        marker.color.g = 0.17;
        marker.color.b = 0.89;
        marker.color.a = 1.0;
        // ----------------------------------------------

        // ---- Marker message with covariance ----
        visualization_msgs::Marker cov_marker;
        cov_marker.header.frame_id = "base_link";
        cov_marker.header.stamp = ros::Time();
        cov_marker.ns = "marker_covariance_list";
        cov_marker.id = 0;
        cov_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        cov_marker.action = visualization_msgs::Marker::ADD;
        // cov_marker.pose.position.x = 0;
        // cov_marker.pose.position.y = 0;
        // cov_marker.pose.position.z = 0;
        cov_marker.pose.orientation.x = 0.0;
        cov_marker.pose.orientation.y = 0.0;
        cov_marker.pose.orientation.z = 0.0;
        cov_marker.pose.orientation.w = 0.0;
        cov_marker.scale.x = SIZE_OF_OBJECT/10;
        cov_marker.scale.y = SIZE_OF_OBJECT/10;
        cov_marker.scale.z = SIZE_OF_OBJECT/10;
        cov_marker.color.r = 0.54;
        cov_marker.color.g = 0.41;
        cov_marker.color.b = 0.8;
        cov_marker.color.a = 0.9;
        // ----------------------------------------------



        // ---- Marker message with global center ----
        visualization_msgs::Marker marker_center;
        marker_center.header.frame_id = "base_link";
        marker_center.header.stamp = ros::Time();
        marker_center.ns = "marker_center_cube_list";
        marker_center.id = 0;
        marker_center.type = visualization_msgs::Marker::CUBE_LIST;
        marker_center.action = visualization_msgs::Marker::ADD;
        // marker.pose.position.x = 0;
        // marker.pose.position.y = 0;
        // marker.pose.position.z = 0;
        marker_center.pose.orientation.x = 0.0;
        marker_center.pose.orientation.y = 0.0;
        marker_center.pose.orientation.z = 0.0;
        marker_center.pose.orientation.w = 0.0;
        marker_center.scale.x = SIZE_OF_OBJECT;
        marker_center.scale.y = SIZE_OF_OBJECT;
        marker_center.scale.z = SIZE_OF_OBJECT;
        marker_center.color.r = 0.93;
        marker_center.color.g = 0.79;
        marker_center.color.b = 0.0;
        marker_center.color.a = 1.0;
        // ----------------------------------------------

        // ---- Marker message with center covariance ----
        visualization_msgs::Marker cov_marker_center;
        cov_marker_center.header.frame_id = "base_link";
        cov_marker_center.header.stamp = ros::Time();
        cov_marker_center.ns = "marker_center_covariance_list";
        cov_marker_center.id = 0;
        cov_marker_center.type = visualization_msgs::Marker::SPHERE_LIST;
        cov_marker_center.action = visualization_msgs::Marker::ADD;
        // cov_marker_center.pose.position.x = 0;
        // cov_marker_center.pose.position.y = 0;
        // cov_marker_center.pose.position.z = 0;
        cov_marker_center.pose.orientation.x = 0.0;
        cov_marker_center.pose.orientation.y = 0.0;
        cov_marker_center.pose.orientation.z = 0.0;
        cov_marker_center.pose.orientation.w = 0.0;
        cov_marker_center.scale.x = SIZE_OF_OBJECT/10;
        cov_marker_center.scale.y = SIZE_OF_OBJECT/10;
        cov_marker_center.scale.z = SIZE_OF_OBJECT/10;
        cov_marker_center.color.r = 0.93;
        cov_marker_center.color.g = 0.46;
        cov_marker_center.color.b = 0.0;
        cov_marker_center.color.a = 0.9;
        // ----------------------------------------------

        std::vector<float> all_x,all_y,all_z,all_cov;
        std::priority_queue<float> all_radius;
        for (PoseWithCovarianceIdentified point : obj->poses)
        {
            all_x.push_back(point.pose.position.x);
            all_y.push_back(point.pose.position.y);
            all_z.push_back(point.pose.position.z);
            all_cov.push_back(point.pose.orientation.w);

            
            geometry_msgs::Point p;
            p.x = point.pose.position.x;
            p.y = point.pose.position.y;
            p.z = point.pose.position.z;

            marker.points.push_back(p);

            cov_marker = Visualiser::drawCovariance(cov_marker,point);


        }
        for (PoseWithCovarianceIdentified point : obj_secondary->poses)
        {
            all_x.push_back(point.pose.position.x);
            all_y.push_back(point.pose.position.y);
            all_z.push_back(point.pose.position.z);
            all_cov.push_back(point.pose.orientation.w);
            
            geometry_msgs::Point p;
            p.x = point.pose.position.x;
            p.y = point.pose.position.y;
            p.z = point.pose.position.z;

            marker.points.push_back(p);
            cov_marker = Visualiser::drawCovariance(cov_marker,point);
       }
       

        float x_avg,y_avg,z_avg,cov_avg,max_radius;

        x_avg = Visualiser::getAverage(all_x);
        y_avg = Visualiser::getAverage(all_y);
        z_avg = Visualiser::getAverage(all_z);
        cov_avg = Visualiser::getAverage(all_cov);

        geometry_msgs::Point p;
        p.x = x_avg;
        p.y = y_avg;
        p.z = z_avg;
        marker_center.points.push_back(p);        
        
        center3D.x = x_avg;
        center3D.y = y_avg;
        center3D.z = z_avg;
        SetMeasurement(center3D);

        cv::Point3f statePt = UpdateKalmanFilter(measurement);
        
        if (PRINT_OUT == 1)
            ROS_INFO_STREAM("Centroid: x "<<x_avg<<" y "<<y_avg<<" z "<<z_avg<<'\n');
        
        cov_matrix = KF.errorCovPost;

        // Calculation of eigen values
        cv::PCA pt_pca(cov_matrix, cv::Mat(), cv::PCA::DATA_AS_ROW, 0);

        //Eigen values and vectors
        cv::Mat pt_eig_vals = pt_pca.eigenvalues;
        cv::Mat pt_eig_vectors = pt_pca.eigenvectors;

        



        l1 = pt_eig_vals.at<float>(0,0);
        l2 = pt_eig_vals.at<float>(1,0);
        l3 = pt_eig_vals.at<float>(2,0);

        // double scale95 = sqrt(5.991);
        R1 = SCALE95 * sqrt(l1);
        R2 = SCALE95 * sqrt(l2);
        R3 = SCALE95 * sqrt(l3);

        PoseWithCovarianceIdentified point;

        point.pose.position.x = x_avg; 
        point.pose.position.y = y_avg; 
        point.pose.position.z = z_avg;


        boost::array<double, 36> cov_parameters{   pt_eig_vectors.at<float>(0,0),pt_eig_vectors.at<float>(1,0),pt_eig_vectors.at<float>(2,0),\
                                        pt_eig_vectors.at<float>(0,1),pt_eig_vectors.at<float>(1,1),pt_eig_vectors.at<float>(2,1),\
                                        pt_eig_vectors.at<float>(0,2),pt_eig_vectors.at<float>(1,2),pt_eig_vectors.at<float>(2,2),\
                                        R1,R2,R3};
        point.covariance = cov_parameters;

        cov_marker_center = Visualiser::drawCovariance(cov_marker_center,point);
        
        // pub_cubes.publish(marker);
        // pub_cubes_cov.publish(cov_marker);
        // pub_center.publish(marker_center);
        // pub_center_cov.publish(cov_marker_center);


        marker_array.markers.push_back(marker);
        marker_array.markers.push_back(cov_marker);
        marker_array.markers.push_back(marker_center);
        marker_array.markers.push_back(cov_marker_center);

        pub_markers.publish(marker_array);
        count++;
    }
};


int main(int argc, char** argv)
{
    ROS_INFO("Visualiser node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_visualiser";


    if(argc < 2) 
    {
        std::cerr<<"Please, enter the drone in the following form: UAV_NAME_OWN UAV_NAME_SEC"<<'\n';
        return 1; 
    }


    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    const char * first  = argv[1];
    const char * second = argv[2];
    // const char * third  = argv[3];
    Visualiser sf(nh,first,second);

    ros::spin();

    return 0;
}
