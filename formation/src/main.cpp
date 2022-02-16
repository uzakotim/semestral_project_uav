// Copyright [2021] [Timur Uzakov]
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>

#include <cmath>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class Formation
{
private:
    ros::NodeHandle nh;


    message_filters::Subscriber<Odometry> sens_fuse_sub;
    message_filters::Subscriber<Odometry> pose_sub;

    typedef sync_policies::ApproximateTime<Odometry,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;


    std::string sens_fuse_sub_topic = "";
    std::string pose_sub_topic      = "";
    std::string pose_pub_topic      = "";


    // parameters
    double n_pos {1.2};
    double n_neg {0.5};
    double delta_max {0.5};
    double delta_min {0.000001};
    double radius {6};

    ros::Publisher pose_pub;
    PoseWithCovarianceStamped msg;


    //
    cv::Mat                 tracker_vector = (cv::Mat_<float>(3,1) << 0,0,0);
    std::vector<cv::Mat>    tracker;

    cv::Mat w_prev = (cv::Mat_<float>(3,1) <<  0,0,0);
    cv::Mat w;
    cv::Mat master_pose;

// measurements
    int count {0};
//

public:


    cv::Mat state,state_cov,object_coord,object_cov;

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
    std::vector<double> grad_cur;
    std::vector<double> grad_prev;

    std::vector<double> delta {0.5,0.5,0.5};
    std::vector<double> delta_prev {0.5,0.5,0.5};
    int k{0};  //computing steps


    Formation(char* name)
    {


        sens_fuse_sub_topic   += "/";
        sens_fuse_sub_topic   += name;
        sens_fuse_sub_topic   += "/sensor_fusion";

        pose_sub_topic += "/";
        pose_sub_topic += name;
        pose_sub_topic += "/odometry/odom_main";

        pose_pub_topic+= "/";
        pose_pub_topic += name;
        pose_pub_topic +="/control_manager/goto";


        sens_fuse_sub.subscribe (nh,sens_fuse_sub_topic,1);
        pose_sub.subscribe      (nh,pose_sub_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sens_fuse_sub,pose_sub));
        sync->registerCallback(boost::bind(&Formation::callback,this,_1,_2));

        tracker.push_back(tracker_vector);
        tracker.push_back(tracker_vector);

        ROS_INFO("All functions initialized");
    }
//    COSTS OF 1ST DRONE : Change here for other drones
    double CostX(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,double radius)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(0) - x_prev.at<float>(0)),2) + std::pow((x.at<float>(0) - master_pose.at<float>(0)+radius),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(object_cov)*10e-4;
        return resulting_cost;
    }
    double CostY(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,double radius)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(1) - x_prev.at<float>(1)),2) + std::pow((x.at<float>(1) - master_pose.at<float>(1)),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(object_cov)*10e-4;
        return resulting_cost;
    }
    double CostZ(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat object_cov,double radius)
    {
        double offset_z {0.5};
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(2) - x_prev.at<float>(2)),2) + std::pow((x.at<float>(2) - master_pose.at<float>(0)-offset_z),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(object_cov)*10e-4;
        return resulting_cost;
    }
    int sign(double x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

     void InitializeCosts()
    {


    }

    void callback(const OdometryConstPtr& object,const OdometryConstPtr& pose)
    {
        ROS_INFO("Synchronized\n");

        // Measurements
        state = (cv::Mat_<float>(4,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,pose->pose.pose.orientation.z);
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
        // ---------------
        // Tracker
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
        // ----------------------
        w = (cv::Mat_<float>(3,1)<< state.at<float>(0),state.at<float>(1),state.at<float>(2));
        // RPROP
        // goal-driven behaviour
        if (master_pose.empty() == false)
        {
            ROS_INFO_STREAM("master at"<<master_pose);
            // run optimization
            // costs
            cost_prev_x = CostX(w_prev,w_prev,master_pose,state_cov,object_cov,radius);
            cost_prev_y = CostY(w_prev,w_prev,master_pose,state_cov,object_cov,radius);
            cost_prev_z = CostZ(w_prev,w_prev,master_pose,state_cov,object_cov,radius);

            cost_cur_x = CostX(w,w_prev,master_pose,state_cov,object_cov,radius);
            cost_cur_y = CostY(w,w_prev,master_pose,state_cov,object_cov,radius);
            cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,object_cov,radius);

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

                cost_cur_x = CostX(w,w_prev,master_pose,state_cov,object_cov,radius);
                cost_cur_y = CostY(w,w_prev,master_pose,state_cov,object_cov,radius);
                cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,object_cov,radius);

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
            ROS_INFO_STREAM("calculated optimal z "<<w.at<float>(2));
            ROS_INFO_STREAM("master pose z "<<master_pose.at<float>(2));
            msg.header.frame_id = "uav1_local_origin";
            msg.header.stamp = ros::Time::now();
            msg.pose.pose.position.x = w.at<float>(0);
            msg.pose.pose.position.y = w.at<float>(1);
            msg.pose.pose.position.z = w.at<float>(2);
            msg.pose.pose.orientation.z = std::round(atan2(master_pose.at<float>(1)-w.at<float>(1),master_pose.at<float>(0)-w.at<float>(0)));

            // Optionally publish error

            pose_pub.publish(msg);

        }
        else
        {
            ROS_INFO_STREAM("calculating "<<count);
        }

        w_prev = (cv::Mat_<float>(3,1)<< state.at<float>(0),state.at<float>(1),state.at<float>(2));
    }

};



int main(int argc, char** argv)
{
    ROS_INFO("Formation node initialized");
    ros::init(argc, argv, "uav1_formation_controller");
    Formation fc(argv[1]);
    ros::spin();

    return 0;
}
