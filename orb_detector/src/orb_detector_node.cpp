// Copyright [2021] [Timur Uzakov]

#include <iostream>
#include <cmath>
// Time Synchronizer Libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Libraries for msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>

// CvBridge, Image Transport
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV libraries
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

// Namespaces
using namespace geometry_msgs;
using namespace message_filters;
using namespace sensor_msgs;

class ORB
{               /*--------------ORB DETECTOR NODE-----------------*/
private:
    ros::NodeHandle                 nh;
    ros::Publisher                  pub;
    ros::Publisher                  pub_obstacles;

    image_transport::Subscriber     sub;
    // Messages
    sensor_msgs::ImagePtr           msg_output;
    sensor_msgs::PointCloud         msg_obstacles;
    // Msgs Topics -------------------------
    std::string                     sub_camera_topic    = "/camera/image"; 
    std::string                     pub_image_topic     = "/ORB/matches";
    std::string                     pub_obstacles_topic = "/ORB/points";
    // Msgs Topics -------------------------

    // ORB variables ---------------
    int                             count {0};
    cv::Mat                         image, image_prev, image_matches;
    const int                       MAX_FEATURES        = 1000;
    const float                     GOOD_MATCH_PERCENT  = 0.15f;
    bool                            flag_first_photo    = true;

    cv::Mat                         descriptors;
    cv::Mat                         descriptors_prev;
    std::vector<cv::KeyPoint>       keypoints;
    std::vector<cv::KeyPoint>       keypoints_prev;
    
    cv::Ptr<cv::Feature2D>          orb     = cv::ORB::create(MAX_FEATURES);
    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch>         matches;   
    // ORB variables ---------------

public:
    ORB()
    {
        image_transport::ImageTransport it(nh);
        sub = it.subscribe(sub_camera_topic,1,&ORB::callback,this);
        pub = nh.advertise<sensor_msgs::Image>(pub_image_topic,1);
        pub_obstacles = nh.advertise<sensor_msgs::PointCloud>(pub_obstacles_topic,1);
        ROS_INFO("ORB Node Initialized Successfully");
    }

    cv::Mat ReturnCVMatImageFromMsg(const sensor_msgs::ImageConstPtr& msg)
    {
        // Function that converts Image msg into cv::Mat format
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            std::cerr<<"Could not convert image into CV Mat\n";
        }
        cv::Mat image = cv_ptr->image;
        return  image;
    }

    std::vector<cv::DMatch> SortMatches (std::vector<cv::DMatch> matches)
    {
        // The function that sortes matches and removes bad ones

        std::sort(matches.begin(), matches.end()); //sort matches by score
        const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
        matches.erase(matches.begin()+numGoodMatches,matches.end()); //remove bad matches
        return matches;
    }

    std::vector<geometry_msgs::Point32> ObtainPoints(std::vector<cv::DMatch> sorted_matches)
    {   
        // The function that obtains feature points 
        // from matches found using ORB

        std::vector<geometry_msgs::Point32>          points_previous;
        std::vector<geometry_msgs::Point32>          points_current;
        cv::Point                                    point_previous;
        cv::Point                                    point_current;
        for (auto& sorted_match : sorted_matches)
        {
                // int index_prev {sorted_matches.queryIdx};
                // point_previous = keypoints_prev.at(index_prev).pt;
                // points_previous.push_back(point_previous);
                int index_cur  {sorted_match.trainIdx};
                point_current  = keypoints.at(index_cur).pt;

                geometry_msgs::Point32 point;
                point.x = point_current.x;
                point.y = point_current.y;
                point.z = 0; // INSERT DEPTH INFORMATION HERE

                points_current.push_back(point);
        }
        return points_current;
        // ROS_INFO_STREAM("[Size of points_current: "<<points_current.size()<<"]");
        // ROS_INFO_STREAM("[x: "<<points_current[0].x<<" y: "<<points_current[0].y<<"]");
    }
    void SendObstaclePoints(std::vector<geometry_msgs::Point32> points_current)
    {
        msg_obstacles.points            = points_current;
        msg_obstacles.header.frame_id   = std::to_string(count);
        msg_obstacles.header.stamp      = ros::Time::now();
        // Publishing
        pub_obstacles.publish(msg_obstacles);
 
    }

    void SendImageMatches(cv::Mat image_matches)
    {
        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matches).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        msg_output->header.stamp    = ros::Time::now();
        //Publishing
        pub.publish(msg_output);
        
    }

    void callback(const ImageConstPtr& msg)
    {
        // ROS_INFO("[Synchronized and ORB started]\n");
        cv::Mat       image           = ReturnCVMatImageFromMsg     (msg);

        if (flag_first_photo == true) 
        {   
            // If the frame is taken for the first time, 
            // compute keypoints and descriptors using ORB,
            // clone image frame for output,
            // set the flag to false
            orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            
            image_matches           = image.clone();
            flag_first_photo        = false;

            ROS_INFO("First Photo\n");
            
        }
        else
        {  
            // If the frame is not first, compute current frame,keypoints and descriptors,
            // match with previously stored keypoints and descriptors,
            // sort and draw them
            
            orb     ->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            matcher ->match           (descriptors_prev, descriptors, matches, cv::Mat());
            // Sorting matches
            std::vector<cv::DMatch>             sorted_matches = SortMatches(matches);
            // Obtaining all points
            std::vector<geometry_msgs::Point32> points_current = ObtainPoints(sorted_matches);
            // Drawing matches
            cv::drawMatches(image_prev, keypoints_prev, image,keypoints, sorted_matches, image_matches);

            SendObstaclePoints(points_current);

        
       }
        // ****************************** 
        SendImageMatches(image_matches);
        // frame update for ORB detection of features
        image_prev          = image.clone();
        keypoints_prev      = keypoints;
        descriptors_prev    = descriptors;
        count++;
    }


};     



int main(int argc, char** argv)
{
    ROS_INFO_STREAM ("Instanciating ORB Detector\n");;
    ros::init(argc, argv, "orb_node");
    ORB orb;
    ros::spin();

    return 0;
}