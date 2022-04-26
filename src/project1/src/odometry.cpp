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

    //Euler integration
    x_k1 = x_k + v_k * t_s * cos(theta_k);
    y_k1 = y_k + v_k * t_s * sen(theta_k);
    theta_k1 = theta_k + w_k * t_s;
    t_s = t_k1 - t_k;

    //Runge-Kutta
    x_k1 = x_k + v_k * t_s * cos(theta_k + (w_k * t_s)/2);
    y_k1 = y_k + v_k * t_s * sen(theta_k + (w_k * t_s)/2);
    theta_k1 = theta_k + w_k * t_s;
    t_s = t_k1 - t_k;

    return 0;
}
