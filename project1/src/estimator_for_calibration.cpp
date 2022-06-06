#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "project1/Rpm.h"
#include "project1/Calibration.h"

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define ENCODERS_RESOLUTION 42
#define GEAR_RATIO 5

class estimator{
public:

    estimator(){
        this->l = WHEEL_POSITION_ALONG_X;
        this->w = WHEEL_POSITION_ALONG_Y;
        this->n = ENCODERS_RESOLUTION;
        this->r = WHEEL_RADIUS;

        this->calibration_pub = this->nh.advertise<project1::Calibration>("calibration", 1000);
    }

   /*void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& gt_pose_msg, const nav_msgs::Odometry& my_pose_msg,
                     const sensor_msgs::JoinState& wheel_state_msg, const project1::Rpm& my_rpm_msg){
        double true_rpm_fl = wheel_state_msg->velocity[0] * (double)(1.0 / 60.0) * (double)(1.0 / GEAR_RATIO) * 9.549297;
        double true_rpm_fr = wheel_state_msg->velocity[1] * (double)(1.0 / 60.0) * (double)(1.0 / GEAR_RATIO) * 9.549297;
        double true_rpm_rl = wheel_state_msg->velocity[2] * (double)(1.0 / 60.0) * (double)(1.0 / GEAR_RATIO) * 9.549297;
        double true_rpm_rr = wheel_state_msg->velocity[3] * (double)(1.0 / 60.0) * (double)(1.0 / GEAR_RATIO) * 9.549297;

        double true_rpm_fl_percent = true_rpm_fl * 0.03;
        double true_rpm_fr_percent = true_rpm_fr * 0.03;
        double true_rpm_rl_percent = true_rpm_rl * 0.03;
        double true_rpm_rr_percent = true_rpm_rr * 0.03;

        double true_quat_x = gt_msg->pose.orientation.x;
        double true_quat_y = gt_msg->pose.orientation.y;
        double true_quat_z = gt_msg->pose.orientation.z;
        double true_quat_w = gt_msg->pose.orientation.w;
        double my_quat_x = my_msg->pose.pose.orientation.x;
        double my_quat_y = my_msg->pose.pose.orientation.y;
        double my_quat_z = my_msg->pose.pose.orientation.z;
        double my_quat_w = my_msg->pose.pose.orientation.w;
        double roll_true,pitch_true,yaw_true,roll_my,pitch_my,yaw_my;
        double true_theta, true_theta_percent, my_theta;

        ROS_INFO("Delta->rpm_fl (true - my) is %f", true_rpm_fl - my_rpm_msg->rpm_fl);
        ROS_INFO("Delta->rpm_fr (true - my) is %f", true_rpm_fr - my_rpm_msg->rpm_fr);
        ROS_INFO("Delta->rpm_rl (true - my) is %f", true_rpm_rl - my_rpm_msg->rpm_rl);
        ROS_INFO("Delta->rpm_rr (true - my) is %f", true_rpm_rr - my_rpm_msg->rpm_rr);

        //se e' troppo lento acceleriamo
        if((true_rpm_fl - true_rpm_fl_percent) > my_rpm_msg->rpm_fl ||
          (true_rpm_fr - true_rpm_fr_percent) > my_rpm_msg->rpm_fr ||
          (true_rpm_rl - true_rpm_rl_percent) > my_rpm_msg->rpm_rl ||
          (true_rpm_rr - true_rpm_rr_percent) > my_rpm_msg->rpm_rr){
            this->n = n - n * 0.05;
            this->r = r + r * 0.02;
        }//se va troppo forte freniamo
        else if((true_rpm_fl + true_rpm_fl_percent) < my_rpm_msg->rpm_fl ||
                (true_rpm_fr + true_rpm_fr_percent) < my_rpm_msg->rpm_fr ||
                (true_rpm_rl + true_rpm_rl_percent) < my_rpm_msg->rpm_rl ||
                (true_rpm_rr + true_rpm_rr_percent) < my_rpm_msg->rpm_rr){
            this->n = n + n * 0.05;
            this->r = r - r * 0.02;
        }

       tf2::Quaternion q_true(true_quat_x,true_quat_y,true_quat_z,true_quat_w);
       tf2::Matrix3x3 m_true(q);
       m_true.getRPY(roll_true,pitch_true,yaw_true);
       tf2::Quaternion q_my(my_quat_x,my_quat_y,my_quat_z,my_quat_w);
       tf2::Matrix3x3 m_my(q);
       m_my.getRPY(roll_my,pitch_my,yaw_my);
       true_theta = yaw_true;
       my_theta = yaw_my;

       true_theta_percent = true_theta * 0.03;

       ROS_INFO("Delta->theta from ground truth pose and my calculated pose is %f", true_theta - my_theta);
        //se theta e' inferiore accorciamo il robot
        if((true_theta - true_theta_percent) > my_theta){
            this->l = l - l * 0.05;
            this->w = w - w * 0.02;
        }//se theta e' superiore allunghiamo il robot
        else if((true_theta + true_theta_percent) < my_theta){
            this->l = l + l * 0.05;
            this->w = w + w * 0.02;
        }

        ROS_INFO("New R: %f N: %f l: %f w: %f", r, n, l , w);

        project1::Calibration calibration_msg;
        calibration_msg.r = this->r;
        calibration_msg.n = this->n;
        calibration_msg.w = this->w;
        calibration_msg.l = this->l;

        calibration_pub.publish(calibration_msg);
    }*/

private:
    ros::NodeHandle nh;
    ros::Publisher calibration_pub;

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
    message_filters::Subscriber <sensor_msgs::JoinState> wheel_states_sub(n, "wheel_states", 1);
    message_filters::Subscriber <project1::Rpm> my_rpm_sub(n, "wheels_rpm", 1);
    TymeSynchronizer <geometry_msgs::PoseStamped, nav_msgs::Odometry, sensor_msgs::JointState, project1::Rpm> sync(gt_pose_sub, my_pose_sub, wheel_states_sub, my_rpm_sub, 10);
    sync.registerCallback(boost::bind(&estimator::poseCallback, _1, _2, _3, _4));*/

    ros::spin();

    return 0;
}