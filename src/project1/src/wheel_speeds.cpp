#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include "project1/Rpm.h"

class wheel_speeds{
public:
    wheel_speeds(){
        this->rpm_pub = this->n.advertise<project1::Rpm>("wheels_rpm",1000);
    }

    void calculateRpm(double v_x, double v_y, double w, ros::Time time){

        // generate  msg
        project1::Rpm rpm_msg;

        rpm_msg.header.stamp = time;

        rpm_msg.rpm_fl = rpm_fl;
        rpm_msg.rpm_fr = rpm_fr;
        rpm_msg.rpm_rl = rpm_rl;
        rpm_msg.rpm_rl = rpm_rr;

        rpm_pub.publish(rpm_msg);
    }

private:
    ros::Publisher rpm_pub;
    double rpm_fl;
    double rpm_fr;
    double rpm_rr;
    double rpm_rl;
};

void cmd_velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    calculateRpm(msg->linear.x,msg->linear.y,msg->angular.z,msg->header.stamp);
}

int main(int argc, char **argv){
    wheel_speeds my_rpm;
    ros::init(argc,argv, "wheel_speeds");
    ros::NodeHandle n;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}
