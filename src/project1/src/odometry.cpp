#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"

void cmd_velCallback(const geometry_msgs::TwistedStamped::ConstPtr& msg) {
    odometry();
}

void odometry(float v_x, float v_y, float w, float time){

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry");  //per inizializzare il nodo
    ros::NodeHandle n;                  //per inizializzare il nodo


    ros::Subscriber odometry_sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);

    ros::spin();      //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni


    return 0;
}
