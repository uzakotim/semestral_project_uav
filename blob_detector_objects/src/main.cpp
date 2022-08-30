// Copyright [2021] [Timur Uzakov]
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mrs_msgs/EstimatedState.h>
#include <nav_msgs/Odometry.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


using namespace geometry_msgs;
using namespace mrs_msgs;
using namespace message_filters;
using namespace nav_msgs;
using namespace sensor_msgs;


#define BLOB_SIZE 15
// 25 - optimal
// 10 - detects UAV motors
#define CAMERA_OFFSET 0.2
#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720
#define RATE 1000

class BlobDetector
{
private:
    // publishers
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::Publisher points_pub;
    // subscribers
    message_filters::Subscriber<Image> image_sub;
    message_filters::Subscriber<Image> depth_sub;
    message_filters::Subscriber<Odometry> pose_sub;
    message_filters::Subscriber<EstimatedState> yaw_sub;

    // synchronization
    typedef sync_policies::ApproximateTime<Image,Image,Odometry,EstimatedState> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // topics
    std::string image_sub_topic       = "";
    std::string depth_sub_topic       = "";
    std::string pose_sub_topic        = "";
    std::string yaw_sub_topic         = "";
    
    std::string points_pub_topic      = "";
    std::string image_pub_topic       = "";

    // detection parameters
    cv::Scalar                  color_min = cv::Scalar(78,158,124);     //BLUE
    cv::Scalar                  color_max = cv::Scalar(138,255,255);    //BLUE

    cv::Scalar                  color_one_min = cv::Scalar(0,70,50);    //RED
    cv::Scalar                  color_one_max = cv::Scalar(10,255,255); //RED
    cv::Scalar                  color_two_min = cv::Scalar(170,70,50);  //RED
    cv::Scalar                  color_two_max = cv::Scalar(180,255,255);//RED
    
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);

public:
    // output parameters
    sensor_msgs::ImagePtr                           msg_output;

    // transforamtion matrices
    cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << IMAGE_WIDTH/2,IMAGE_HEIGHT/2,0); 
                                //! Always check image size depending on camera
 
    cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) <<  0.005,0,0, 
                                                            0,-0.005,0, 
                                                            0,0,-1);
                                //x same, y flip, flip z and rescale
 
    // Kalman Filter parameters
    cv::KalmanFilter KF = cv::KalmanFilter(6,3,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(3,1);

    cv::Mat state, object_coord,object_cov,object_world;

    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    // auxilary parameters
    float yaw_value;
    int counter{0};

    BlobDetector(char* name)
    {
    // INITIALIZATION -------------------------------------------------------------
        //publishers
        image_pub_topic  += "/";
        image_pub_topic  += name;
        image_pub_topic  += "/blob_detection_objects";

        points_pub_topic += "/";
        points_pub_topic += name;
        points_pub_topic += "/points/";

        image_pub          = nh.advertise<Image>(image_pub_topic, 1);
        points_pub         = nh.advertise<PoseArray>(points_pub_topic, 1);


        //subscribers
        image_sub_topic += "/";
        image_sub_topic += name;
        image_sub_topic +="/rgbd_down/color/image_raw";

        depth_sub_topic  += "/";
        depth_sub_topic  += name;
        depth_sub_topic  += "/rgbd_down/aligned_depth_to_color/image_raw";

        pose_sub_topic += "/";
        pose_sub_topic += name;
        pose_sub_topic +="/odometry/odom_main/";

        yaw_sub_topic += "/";
        yaw_sub_topic += name;
        yaw_sub_topic +="/odometry/heading_state_out";


        image_sub.subscribe(nh,image_sub_topic,1);
        depth_sub.subscribe(nh,depth_sub_topic,1);
        pose_sub.subscribe(nh,pose_sub_topic,1);
        yaw_sub.subscribe(nh,yaw_sub_topic,1);

        //synchronization
        sync.reset(new Sync(MySyncPolicy(10), image_sub,depth_sub,pose_sub,yaw_sub));
        sync->registerCallback(boost::bind(&BlobDetector::callback,this,_1,_2,_3,_4));

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
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(10)); 
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----
        ROS_INFO("All functions initialized");
    }

    void PrintThatMessageWasReceived(const std::string& frame_id)
    {
        ROS_INFO_STREAM("[Image from: " <<frame_id<<" ]");
    }

    void PredictUsingKalmanFilter()
    {
        // Prediction, to update the internal statePre variable 
        cv::Mat prediction  =   KF.predict();
        cv::Point3d predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
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

    cv::Mat ReturnBlueMask(cv::Mat image)
    {
        cv::Mat          image_threshold;
        cv::inRange     (image, BlobDetector::color_min, BlobDetector::color_max, image_threshold);
        return image_threshold;
    }

    cv::Mat ReturnRedMask(cv::Mat image)
    {
        cv::Mat          mask1,mask2,total;
        cv::inRange     (image, BlobDetector::color_one_min, BlobDetector::color_one_max, mask1);
        cv::inRange     (image, BlobDetector::color_two_min, BlobDetector::color_two_max, mask2);

        total = mask1 | mask2;
        
        return total;
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

    int FindMaxAreaContourId(std::vector<std::vector<cv::Point>> contours)
    {
        // Function for finding maximal size contour
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
    
    cv::Mat ObjectCoordinateToWorld(cv::Mat object_position,float yaw_value,cv::Mat drone_position,cv::Mat offset_vector)
    {
        // Function for transforming object position to global frame
        cv::Mat shifted_and_scaled  = scale_matrix * (object_position - shift_to_center);   
        cv::Mat RotationMatrix4     = (cv::Mat_<float>(3,3) << cos(yaw_value - M_PI/2),-sin(yaw_value - M_PI/2),0, sin(yaw_value-M_PI/2),cos(yaw_value-M_PI/2),0,  0,0,1) ;                 
        cv::Mat rotated_vector      = RotationMatrix4 * shifted_and_scaled;
        cv::Mat point = drone_position + rotated_vector + offset_vector; 
        return point;
    }


    void callback(const ImageConstPtr& msg,const ImageConstPtr& depth_msg, const OdometryConstPtr& pose, const EstimatedStateConstPtr& yaw)
    {
        ros::Rate rate(RATE);
        ROS_INFO("Synchronized\n");

        std_msgs::Header    msg_header  = depth_msg->header;
        std::string         frame_id    = msg_header.frame_id;
        
        cv::Mat cv_image     = ReturnCVMatImageFromMsg     (msg);
        cv::Mat depth_image  = ReturnCVMatImageFromDepthMsg(depth_msg);

        std::vector<Pose> points_array {}; 
        // -->> Operations on image ----
        cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
        
        // 1) smoothing
        //cv::Mat     blurred_image   = GaussianBlur(cv_image);
        
        // 2) conversion to hsv
        cv::Mat     image_HSV       = BGRtoHSV(cv_image);
        // 3) finding orange mask
        cv::Mat     image_threshold = ReturnBlueMask(image_HSV);
        // 4) finding contours
        std::vector<std::vector<cv::Point>> contours = ReturnContours(image_threshold);
        
        // Image for detections
        cv::Mat drawing = cv::Mat::zeros(cv_image.size(), CV_8UC3 );
        
        //---------------------MEASUREMENTS---------------------------
        state           = (cv::Mat_<float>(3,1)<< pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z);
        yaw_value       = (float)(yaw->state[0]);
        cv::Mat offset  = (cv::Mat_<float>(3,1) << (CAMERA_OFFSET*cos(yaw_value)),(CAMERA_OFFSET*sin(yaw_value)),0); // 0.2
        
        for (size_t i = 0;i<contours.size();i++)
        {
                double   newArea = cv::contourArea(contours.at(i));
                if(newArea > BLOB_SIZE)
                {   

                    PredictUsingKalmanFilter();
                    // Finding blob's center       
                    cv::Point2f center = FindCenter(contours, i);
                    float       radius = FindRadius(contours, i);

                    cv::Point3d center3D;
                    center3D.x = center.x;
                    center3D.y = center.y;
                    unsigned short val = depth_image.at<unsigned short>(center.y, center.x);
                    center3D.z = (float)val;
                    center3D.z /= 1000.0;
                    // Filering                    
                    SetMeasurement(center3D);
                    cv::Point3d statePt = UpdateKalmanFilter(measurement);
                    cov_matrix = KF.errorCovPost;

                    // Drawing Point
                    cv::Point2d statePt2D;
                    statePt2D.x = center3D.x;
                    statePt2D.y = center3D.y;
                    cv::circle  (drawing, statePt2D, int(radius), detection_color, 2 );
                    cv::circle  (drawing, statePt2D, 5, detection_color, 10);
                    // Conversion to global coordinates
                    object_coord    = (cv::Mat_<float>(3,1)<< (float)statePt.x, (float)statePt.y, (float)statePt.z);
                    object_world = ObjectCoordinateToWorld(object_coord,yaw_value,state,offset);
                    ROS_INFO_STREAM("[OBJECT " << i << " ]:" << object_world.at<float>(0)<<" | " << object_world.at<float>(1)<< " | "<< object_world.at<float>(2));
                
                    // Adding point to array
                    Pose point;
                    point.position.x = object_world.at<float>(0);
                    point.position.y = object_world.at<float>(1);
                    point.position.z = object_world.at<float>(2);
                    point.orientation.w = cv::determinant(cov_matrix)*10e-6;
                    points_array.push_back(point);
                }else
                {
                    Pose point;
                    point.position.x = '\0';
                    point.position.y = '\0';
                    point.position.z = '\0';
                    point.orientation.w = '\0';
                    points_array.push_back(point);
                }
        }

        // ---------------------MSG-----------------------------------------------
        cv::Mat display = cv_image + drawing;
        msg_output= cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
        msg_output->header.frame_id = std::to_string(counter);
        msg_output->header.stamp = ros::Time::now();
        image_pub.publish(msg_output);
        // ---------------------MSG-----------------------------------------------
        geometry_msgs::PoseArray msg_points;
        msg_points.poses = points_array;
        msg_points.header.frame_id = std::to_string(counter);
        msg_points.header.stamp = ros::Time::now();
        points_pub.publish(msg_points);

        rate.sleep();
        counter++; 
    }
};



int main(int argc, char** argv)
{
    ROS_INFO("BlobDetector node initialized");
    std::string node_name = "";
    node_name += argv[1];
    node_name += "_blob_detector_objects";

    ros::init(argc, argv, node_name);
    BlobDetector bd(argv[1]);
    ros::spin();

    return 0;
}
