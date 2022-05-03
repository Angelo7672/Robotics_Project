#include "ros/ros.h"
#include "project1/Reset.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "reset_client");

    if (argc != 4){
        ROS_INFO("usage: reset_client new_x new_y new_theta");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1::Reset>("reset");
    project1::Reset srv;
    srv.request.new_x = atof(argv[1].c_str());
    srv.request.new_y = atof(argv[2].c_str());
    srv.request.new_theta = atof(argv[3].c_str());

    if (client.call(srv)){
        ROS_INFO("Old odom: (x:%f,y:%f,theta:%f)", (double)srv.response.old_x, (double)srv.response.old_y, (double)srv.response.old_theta);
    } else {
        ROS_ERROR("Failed to call service reset");
        return 1;
    }

    return 0;
}