#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include "project1/Rpm.h"
#include "project1/Calibration.h"

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define ENCODERS_RESOLUTION 42

class wheel_speeds{
public:
    wheel_speeds(){
        this->rpm_pub = this->nh.advertise<project1::Rpm>("wheels_rpm",1000);
        this->r = 0.077/*WHEEL_RADIUS*/;
        this->n = 43/*ENCODERS_RESOLUTION*/;
        this->w = 0.17/*WHEEL_POSITION_ALONG_Y*/;
        this->l = 0.18/*WHEEL_POSITION_ALONG_X*/;
    }

    void calculateRpm(double v_x, double v_y, double w_z, ros::Time time){
        rpm_fl = ((1/r) * ((-l - w) * w_z + v_x - v_y)) * 9.549297;
        rpm_fr = ((1/r) * ((l + w) * w_z + v_x + v_y)) * 9.549297;
        rpm_rl = ((1/r) * ((l + w) * w_z + v_x - v_y)) * 9.549297;
        rpm_rr = ((1/r) * ((-l - w) * w_z + v_x + v_y)) * 9.549297;

        // generate  msg
        project1::Rpm rpm_msg;

        rpm_msg.header.stamp = time;

        rpm_msg.rpm_fl = rpm_fl;
        rpm_msg.rpm_fr = rpm_fr;
        rpm_msg.rpm_rl = rpm_rl;
        rpm_msg.rpm_rr = rpm_rr;

        rpm_pub.publish(rpm_msg);
    }

    void cmd_velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        calculateRpm(msg->twist.linear.x,msg->twist.linear.y,msg->twist.angular.z,msg->header.stamp);
    }

    void calibrationCallback(const project1::Calibration::ConstPtr& msg){
        this->r = msg->r;
        this->n = msg->n;
        this->l = msg->l;
        this->w = msg->w;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher rpm_pub;

    double rpm_fl;
    double rpm_fr;
    double rpm_rl;
    double rpm_rr;

    double r;
    double n;
    double l;
    double w;
};

int main(int argc, char **argv){
    ros::init(argc,argv, "wheel_speeds");
    ros::NodeHandle n;

    wheel_speeds my_rpm;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, &wheel_speeds::cmd_velCallback, &my_rpm);
    ros::Subscriber calibration_sub = n.subscribe("calibration", 1000, &wheel_speeds::calibrationCallback, &my_rpm);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}