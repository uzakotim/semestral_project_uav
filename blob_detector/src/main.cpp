// Copyright [2021] [Timur Uzakov]
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class BlobDetector
{
private:
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::Publisher object_coord_pub;

    message_filters::Subscriber<Image> image_sub;
    message_filters::Subscriber<Image> depth_sub;

    typedef sync_policies::ApproximateTime<Image,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


    std::string image_sub_topic       = "";
    std::string depth_sub_topic       = "";
    std::string image_pub_topic       = "";
    std::string object_coord_pub_topic = "";



    double object_x = 0;
    double object_y = 0;
    double object_z = 0;

    cv::Scalar                  color_min = cv::Scalar(78,158,124); //BLUE
    cv::Scalar                  color_max = cv::Scalar(138,255,255); //BLUE
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);



public:
    // Output Parameters
    sensor_msgs::ImagePtr                          msg_output;
    nav_msgs::Odometry                              msg_object;

    int counter = 0;


    cv::KalmanFilter KF = cv::KalmanFilter(6,3,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(3,1);

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);


    BlobDetector(char* name)
    {


        image_pub_topic  += "/";
        image_pub_topic  += name;
        image_pub_topic  += "/blob_detection";

        object_coord_pub_topic += "/";
        object_coord_pub_topic += name;
        object_coord_pub_topic += "/object/";

        image_sub_topic += "/";
        image_sub_topic += name;
        image_sub_topic +="/rgbd/color/image_raw";

        depth_sub_topic  += "/";
        depth_sub_topic  += name;
        depth_sub_topic  += "/rgbd/aligned_depth_to_color/image_raw";

        image_sub.subscribe(nh,image_sub_topic,1);
        depth_sub.subscribe(nh,depth_sub_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), image_sub,depth_sub));
        sync->registerCallback(boost::bind(&BlobDetector::callback,this,_1,_2));

        image_pub         = nh.advertise<Image>(image_pub_topic, 1);
        object_coord_pub   = nh.advertise<Odometry>(object_coord_pub_topic, 1);

        //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(6,6) <<  1,0,0,1,0,0,
                                                        0,1,0,0,1,0,
                                                        0,0,1,0,0,1,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        measurement.setTo(cv::Scalar(0));
        KF.statePre.at<float>(0) = 0;
        KF.statePre.at<float>(1) = 0;
        KF.statePre.at<float>(2) = 0;
        KF.statePre.at<float>(3) = 0;
        KF.statePre.at<float>(4) = 0;
        KF.statePre.at<float>(5) = 0;

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1e-4));
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----
        ROS_INFO("All functions initialized");
    }
    void PrintThatMessageWasReceived(std::string frame_id)
    {

        ROS_INFO_STREAM("[Image from: " <<frame_id<<" ]");
    }
    cv::Point3d PredictUsingKalmanFilter()
    {
        // Prediction, to update the internal statePre variable -->>
        cv::Mat prediction  =   KF.predict();
        cv::Point3d predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
        return  predictPt;
        //  <<---Prediction, to update the internal statePre variable
    }
    cv::Mat ReturnCVMatImageFromMsg(const sensor_msgs::ImageConstPtr& msg)
    {
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

    cv::Mat ReturnCVMatImageFromDepthMsg(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            std::cerr<<"Could not convert depth image into CV Mat\n";
        }
        cv::Mat image = cv_ptr->image;
        return  image;
    }
    cv::Mat GaussianBlur(cv::Mat image)
    {
        cv::Mat image_blurred;
        cv::GaussianBlur(image, image_blurred, cv::Size(5,5), 0);
        return  image_blurred;
    }
    cv::Mat BGRtoHSV(cv::Mat image)
    {
        cv::Mat image_HSV;
        cv::cvtColor(image, image_HSV,CV_BGR2HSV);
        return  image_HSV;
    }
    cv::Mat ReturnOrangeMask(cv::Mat image)
    {
        cv::Mat          image_threshold;
        cv::inRange     (image, BlobDetector::color_min, BlobDetector::color_max, image_threshold);
        return image_threshold;
    }
    std::vector<std::vector<cv::Point>> ReturnContours(cv::Mat image_threshold)
    {
        std::vector<std::vector<cv::Point>> contours;       //contours are stored here
        std::vector<cv::Vec4i>              hierarchy;
        cv::findContours(image_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }
    cv::Point2f FindCenter(std::vector<std::vector<cv::Point>> contours, int ID)
    {
        std::vector<cv::Point>  contour_poly   (contours.size());
        cv::Point2f             center         (contours.size());
        float                   radius         (contours.size());

        cv::approxPolyDP        (contours[ID], contour_poly, 3, true);
        cv::minEnclosingCircle  (contour_poly, center, radius);
        return center;

    }
    float FindRadius(std::vector<std::vector<cv::Point>> contours, int ID)
    {
        std::vector<cv::Point>  contour_poly   (contours.size());
        cv::Point2f             center         (contours.size());
        float                   radius         (contours.size());

        cv::approxPolyDP        (contours[ID], contour_poly, 3, true);
        cv::minEnclosingCircle  (contour_poly, center, radius);
        return radius;

    }
    void SetMeasurement(cv::Point3d center)
    {
        measurement.at<float>(0) = center.x;
        measurement.at<float>(1) = center.y;
        measurement.at<float>(2) = center.z;
    }
    cv::Point3d UpdateKalmanFilter(cv::Mat_<float>  measurement)
    {
        // Updating Kalman Filter
        cv::Mat       estimated = KF.correct(measurement);
        cv::Point3d   statePt(estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
        cv::Point3d   measPt(measurement(0),measurement(1),measurement(2));
        return      statePt;
    }

     // Function for finding maximal size contour
    int FindMaxAreaContourId(std::vector<std::vector<cv::Point>> contours)
    {
        double  maxArea          = 0;
        int     maxAreaContourId = -1;

        for (size_t i = 0;i<contours.size();i++)
        {
                double   newArea = cv::contourArea(contours.at(i));
                if(newArea > maxArea)
                {
                        maxArea = newArea;
                        maxAreaContourId = i;
                }
        }
        return maxAreaContourId;
    }
    void callback(const ImageConstPtr& msg,const ImageConstPtr& depth_msg)
    {
        ROS_INFO("Synchronized\n");
        std_msgs::Header    msg_header  = depth_msg->header;
        std::string         frame_id    = msg_header.frame_id;
        PrintThatMessageWasReceived (frame_id);

        cv::Mat cv_image     = ReturnCVMatImageFromMsg     (msg);
        cv::Mat depth_image  = ReturnCVMatImageFromDepthMsg(depth_msg);




        cv::Point3d predictPt= PredictUsingKalmanFilter();
        // -->> Operations on image ----
        cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
        // 1) smoothing the image

        cv::Mat     blurred_image   = GaussianBlur(cv_image);
        // 2) conversion to hsv
        cv::Mat     image_HSV       = BGRtoHSV(blurred_image);
        // 3) finding orange mask
        cv::Mat     image_threshold = ReturnOrangeMask(image_HSV);
        // 4) finding contours
        std::vector<std::vector<cv::Point>> contours = ReturnContours(image_threshold);
        // 5) finding max contour
        int maxAreaContourId        = BlobDetector::FindMaxAreaContourId(contours);

        cv::Mat drawing = cv::Mat::zeros(cv_image.size(), CV_8UC3 );
        // 6,7) finding center and kalman filtering

        if (maxAreaContourId>=0)
        {

            cv::Point2f center = FindCenter(contours, maxAreaContourId);
            float       radius = FindRadius(contours, maxAreaContourId);

            cv::Point3d center3D;

            center3D.x = center.x; //from left to right
            center3D.y = center.y; // from top to bottom
            unsigned short val = depth_image.at<unsigned short>(center.y, center.x);
            center3D.z = static_cast<float>(val);
            center3D.z /= 1000.0;

            // uncomment the following to draw contours exactly
              // cv::drawContours(drawing, contours, maxAreaContourId,
                //                detectionColor ,5, cv::LINE_8, hierarchy, 0 );

            // uncomment the following for checking center coordinates
            // std::cout<<center<<'\n';
            // std::cout<<center3D.z<<'\n';


             //-<<---Blob detector
            // Obtaining the point from Kalman Filter
            SetMeasurement(center3D);



            cv::Point3d statePt = UpdateKalmanFilter(measurement);


            // uncomment the following for checking estimated center coordinates
            // std::cout<<statePt<<'\n';
            cv::Point2d statePt2D;
            statePt2D.x = statePt.x;
            statePt2D.y = statePt.y;
             // Drawing Point
            cv::circle  (drawing, statePt2D, int(radius), detection_color, 2 );
            cv::circle  (drawing, statePt2D, 5, detection_color, 10);

            msg_object.pose.pose.position.x = (double)statePt.x;
            msg_object.pose.pose.position.y = (double)statePt.y;
            msg_object.pose.pose.position.z = (double)statePt.z;
            // Set covariance

            cov_matrix = KF.errorCovPost;

            ROS_INFO_STREAM("[Detected blue object: z "<< center3D.z <<"]");
            
            cv::Mat display = cv_image + drawing;
            msg_output= cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
            msg_output->header.frame_id = std::to_string(counter);
            msg_output->header.stamp = ros::Time::now();
            image_pub.publish(msg_output);

            msg_object.pose.covariance = msg_cov_array;
            msg_object.header.frame_id = std::to_string(counter);
            msg_object.header.stamp = ros::Time::now();
            object_coord_pub.publish(msg_object);

            
        }
        else    {
            cv::Mat display = cv_image;
            msg_output= cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
            msg_output->header.frame_id = std::to_string(counter);
            msg_output->header.stamp = ros::Time::now();
            image_pub.publish(msg_output);
            
            msg_object.pose.pose.position.x = NULL;
            msg_object.pose.pose.position.y = NULL;
            msg_object.pose.pose.position.z = NULL;
            msg_object.pose.covariance = msg_cov_array;
            msg_object.header.frame_id = std::to_string(counter);
            msg_object.header.stamp = ros::Time::now();
            object_coord_pub.publish(msg_object);
        }
        counter++; 
    }
};



int main(int argc, char** argv)
{
    ROS_INFO("BlobDetector node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_blob_detector";

    ros::init(argc, argv, node_name);
    BlobDetector bd(argv[1]);
    ros::spin();

    return 0;
}
