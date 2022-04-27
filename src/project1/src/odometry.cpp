#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"
#include <string>

class odometry_class(){
public:
    int seq;
    string frame_id;
    int time_sec;
    int time_nsec;
    float v_x;
    float v_y;
    float w_z;

    int t_k_sec;
    int t_k1_sec;
    int t_k_nsec;
    int t_k1_nsec;

    odometry_class(){ }


    long double evaluate_Ts(int tsec, int tnsec, int t1sec, int t1nsec){
        long double returnedValue;
        returnedValue = ((long double) (t1sec - tsec)) + ((long double)(t1nsec - tnsec))/1000000000;
        return returnedValue;
    }
}

void cmd_velCallback(const geometry_msgs::TwistedStamped::ConstPtr& msg) {
    odometry(msg->header.seq,msg->header.frame_id,msg->header.stamp.sec,msg->header.stamp.nsec,msg->linear.x,msg->linear.y,msg->angular.z);
}

void odometry(int seq, string frame_id, int tsec, int tnsec, float v_x, float v_y, float w){
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
}

int main(int argc, char **argv) {
    odometry_class my_odometry;
    ros::init(argc, argv, "odometry");  //per inizializzare il nodo
    ros::NodeHandle n;                  //per inizializzare il nodo

    ros::Subscriber odometry_sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);

    ros::spin();      //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}
