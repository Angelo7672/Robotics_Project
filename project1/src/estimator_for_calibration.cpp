#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define ENCODERS_RESOLUTION 42

class estimator{
public:

    estimator(){
        this->l = WHEEL_POSITION_ALONG_X;
        this->w = WHEEL_POSITION_ALONG_Y;
        this->n = ENCODERS_RESOLUTION;
        this->r = WHEEL_RADIUS;
    }

   /* void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& gt_msg, const nav_msgs::Odometry& my_msg){
        double delta_x = gt_msg->pose.position.x - my_msg->pose.pose.position.x;
        double delta_y = gt_msg->pose.position.y - my_msg->pose.pose.position.y;
        double delta_quat_x = gt_msg->pose.orientation.x - my_msg->pose.pose.orientation.x;
        double delta_quat_y = gt_msg->pose.orientation.x - my_msg->pose.pose.orientation.y;
        double delta_quat_z = gt_msg->pose.orientation.x - my_msg->pose.pose.orientation.z;
        double delta_quat_w = gt_msg->pose.orientation.x - my_msg->pose.pose.orientation.w;

        ROS_INFO("Delta->x from ground truth pose and my calculated pose is %f", delta_x);
        if(gt_msg->pose.position.x / my_msg->pose.pose.position.x > (double)(3.0 / 100.0)){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }else if(my_msg->pose.pose.position.x / gt_msg->pose.position.x > (double)(3.0 / 100.0)){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

        ROS_INFO("Delta->y from ground truth pose and my calculated pose is %f\n", delta_y);
        if(gt_msg->pose.position.y / my_msg->pose.pose.position.y > (double)(3.0 / 100.0)){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }else if(my_msg->pose.pose.position.y / gt_msg->pose.position.y > (double)(3.0 / 100.0)){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

       ROS_INFO("Delta->quat_x from ground truth pose and my calculated pose is %f", delta_quat_x);
        if(gt_msg->pose.position.x / my_msg->pose.pose.position.x > (double)(3.0 / 100.0)){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }else if(my_msg->pose.pose.position.x / gt_msg->pose.position.x > (double)(3.0 / 100.0)){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

        ROS_INFO("Delta->quat_x from ground truth pose and my calculated pose is %f", delta_quat_x);
        if(gt_msg->pose.position.x / my_msg->pose.pose.position.x > (double)(3.0 / 100.0)){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }else if(my_msg->pose.pose.position.x / gt_msg->pose.position.x > (double)(3.0 / 100.0)){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

        ROS_INFO("Delta->quat_x from ground truth pose and my calculated pose is %f", delta_quat_x);
        if(gt_msg->pose.position.x / my_msg->pose.pose.position.x > (double)(3.0 / 100.0)){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }else if(my_msg->pose.pose.position.x / gt_msg->pose.position.x > (double)(3.0 / 100.0)){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

        ROS_INFO("Delta->quat_x from ground truth pose and my calculated pose is %f", delta_quat_x);
        if(gt_msg->pose.position.x / my_msg->pose.pose.position.x > (double)(3.0 / 100.0)){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }else if(my_msg->pose.pose.position.x / gt_msg->pose.position.x > (double)(3.0 / 100.0)){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

    }*/



private:
    double n;
    double l;
    double w;
    double r;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibrator");  //per inizializzare il nodo
    ros::NodeHandle n;                    //per inizializzare il nodo

    estimator my_estimator;

    /*message_filters::Subscriber <geometry_msgs::PoseStamped> gt_pose_sub(n, "robot/pose", 1);
    message_filters::Subscriber <nav_msgs::Odometry> my_pose_sub(n, "odom", 1);
    TymeSynchronizer <geometry_msgs::PoseStamped, nav_msgs::Odometry> sync(gt_pose_sub, my_pose_sub, 10);
    sync.registerCallback(boost::bind(&estimator::poseCallback, _1, _2));*/

    ros::spin();

    return 0;
}


