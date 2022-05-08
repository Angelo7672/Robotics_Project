#include "ros/ros.h"
#include "project1/Reset.h"
#include <tf2/LinearMath/Matrix3x3.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "reset_client");

    double roll,pitch,yaw;

    if (argc != 7){
        ROS_INFO("usage: reset_client new_x new_y new_quat_x new_quat_y new_quat_z new_quat_w");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1::Reset>("reset");

    project1::Reset srv;
    srv.request.new_x = atof(argv[1]);
    srv.request.new_y = atof(argv[2]);
    double quat_x = atof(argv[3]);
    double quat_y = atof(argv[4]);
    double quat_z = atof(argv[5]);
    double quat_w = atof(argv[6]);

    tf2::Quaternion q(quat_x,quat_y,quat_z,quat_w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);

    srv.request.new_theta = yaw;

    if (client.call(srv)){
        ROS_INFO("Old odom: (x:%f,y:%f,theta:%f)", (double)srv.response.old_x, (double)srv.response.old_y, (double)srv.response.old_theta);
    } else {
        ROS_ERROR("Failed to call service reset");
        return 1;
    }

    return 0;
}