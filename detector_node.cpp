// Copyright [2021] [Timur Uzakov]

// include message filters and time sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>

// include CvBridge, Image Transport, Image msg

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>
#include <string.h>

using namespace sensor_msgs;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace message_filters;

class BlobDetector 
{
private:
    // Publishers 
    image_transport::Publisher  pub;
    ros            ::Publisher  pub_point;
    image_transport::Subscriber sub;
    // Blob Detector Parametrs
    cv::Scalar                  orange_min = cv::Scalar(10,110,110);     //min hsv value orange
    cv::Scalar                  orange_max = cv::Scalar(27,255,255);     //max hsv value orange
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);

    int count = 0;
    geometry_msgs::PointStamped goal;

    // Output Parameters
    sensor_msgs::ImagePtr       msg_output;
    std::string sub_topic_image = "";
    std::string sub_topic_depth = "";
    std::string pub_topic = "";
    

    message_filters::Subscriber<Image> sub_1;
    message_filters::Subscriber<Image> sub_2;
    typedef sync_policies::ApproximateTime<Image,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    cv::KalmanFilter KF = cv::KalmanFilter(4,2,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(2,1);

    // Constructor
    BlobDetector(ros::NodeHandle *nh,char* name)
    {           
        pub_topic += "/";
        pub_topic += name;
        pub_topic +="/camera/blob";
                 
        sub_topic_image += "/";
        sub_topic_image += name;
        sub_topic_image +="/rgbd/color/image_raw";
        
        sub_topic_depth += "/";
        sub_topic_depth += name;
        sub_topic_depth +="/rgbd/aligned_depth_to_color/image_raw";
        
        image_transport::ImageTransport it(*nh);

        pub = it.advertise(pub_topic, 1);
        pub_point   = nh->advertise<geometry_msgs::PointStamped>("/camera/goal", 1);

        sub_1.subscribe(*nh,sub_topic_image,1);
        sub_2.subscribe(*nh,sub_topic_depth,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&BlobDetector::image_callback,this,_1,_2));

        ROS_INFO("All functions initialized");


    //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(6,6) << 1,0,0,1,0,0, 0,1,0,0,1,0, 0,0,1,0,0,1, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1 );
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
        ROS_INFO("Blob Detector Initialized Successfully");
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
    void PrintThatMessageWasReceived(std::string frame_id)
    {
        
        ROS_INFO_STREAM("[Image from: " <<frame_id<<" ]");
    }
    cv::Point3f PredictUsingKalmanFilter()
    {
        // Prediction, to update the internal statePre variable -->>
        cv::Mat prediction  =   KF.predict();
        cv::Point3f predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
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
    cv::Mat ReturnMask(cv::Mat image)
    {

        cv::Mat mask1, mask2;
        cv::inRange(image, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(image, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);

        cv::Mat mask = mask1 | mask2;

        return mask;
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


    void SetMeasurement(cv::Point2f center,uint8[] image_depth)
    {
        measurement.at<float>(0) = center.x;
        measurement.at<float>(1) = center.y;
        measurement.at<float>(2) = (image_depth[(int)center.x][(int)center.y])/1000.0;
    }
    cv::Point3f UpdateKalmanFilter(cv::Mat_<float>  measurement)
    {
        // Updating Kalman Filter    
        cv::Mat     estimated = KF.correct(measurement);
        cv::Point3f statePt(estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
        cv::Point3f measPt(measurement(0),measurement(1),measurement(2));
        return      statePt;
    }
    void SetGoal(cv::Point3f statePt)
    {
        goal.point.x = statePt.x;
        goal.point.y = statePt.y; 
        goal.point.z = statePt.z;
    }
    // Callback for received camera frame
    void image_callback(const ImageConstPtr& msg,const ImageConstPtr& msg_depth)
    {   
        std_msgs::Header    msg_header  = msg->header;
        std::string         frame_id    = msg_header.frame_id;
    
        
        PrintThatMessageWasReceived (frame_id);
        cv::Point3d  predictPt       = PredictUsingKalmanFilter    ();
        cv::Mat     image        = ReturnCVMatImageFromMsg     (msg);
        
        ROS_INFO_STREAM(msg_depth->data);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
         // -->> Operations on image ----
        // 1) smoothing the image
        cv::Mat     image_blurred   = GaussianBlur(image);
        // 2) conversion to hsv
        cv::Mat     image_HSV       = BGRtoHSV(image_blurred); 
        // 3) finding orange mask
        cv::Mat     image_threshold = ReturnMask(image_HSV);

        // 4) finding contours
        std::vector<std::vector<cv::Point>> contours;       //contours are stored here
        std::vector<cv::Vec4i>              hierarchy; 
        cv::findContours(image_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // std::vector<std::vector<cv::Point>> contours = ReturnContours(image_threshold);

        // 5) finding max contour
        int maxAreaContourId        = BlobDetector::FindMaxAreaContourId(contours);
        
        cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3 );
        // 6,7) finding center and kalman filtering
        if (maxAreaContourId>=0)
        {   
            
            cv::Point2f center = FindCenter(contours, maxAreaContourId);
            float       radius = FindRadius(contours, maxAreaContourId);

            // uncomment the following to draw contours exactly
            cv::drawContours(drawing, contours, maxAreaContourId, 
                                 cv::Scalar(255,100,0) ,5, cv::LINE_8, hierarchy, 0 );


            // uncomment the following for checking center coordinates
            // std::cout<<center<<'\n';

             //-<<---Blob detector
            
            // Obtaining the point from Kalman Filter
            SetMeasurement(center,msg_depth->data);
            
            cv::Point3f statePt = UpdateKalmanFilter(measurement);
            
            // uncomment the following for checking estimated center coordinates
            // std::cout<<statePt<<'\n';

               
            
             // Drawing Point
            cv::Point pointToDraw((int)statePt.x, (int)statePt.y);

            cv::circle  (drawing, pointToDraw, int(radius), detection_color, 2 );
            cv::circle  (drawing, pointToDraw, 5, detection_color, 10);   
            
            // Setting up goal point
            SetGoal(statePt); 
            goal.header.stamp = ros::Time::now();
            pub_point.publish(goal);
 
            // ROS_INFO_STREAM("[Detected red object: x "<< statePt.x<<" y "<<statePt.y<<" z " <<statePt.z <<"]"); 

        }

    cv::Mat display = image + drawing;
    msg_output= cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
    msg_output->header.frame_id = std::to_string(count);
    
    msg_output->header.stamp = ros::Time::now();
    
   
    
    count++;


    pub.publish(msg_output);
    
    // uncomment for debugging
    //ROS_INFO_STREAM("[Image:" << frame_id<<" was sent ]");

    }
};




int main(int argc, char** argv)
{
    ROS_INFO_STREAM  (argv[1]);
    ros::init        (argc, argv, "roscpp_blob_detector");
    ros::NodeHandle  nh;
    BlobDetector     bd = BlobDetector(&nh, argv[1]);
    ros::spin();
    return 0;
}
