#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"
#include <string>


void cmd_velCallback(const geometry_msgs::TwistedStamped::ConstPtr& msg) {
    odometry();
}
class odometryClass(){
    public:

    odometryClass(){

    }





    long double evaluateT_s(int tsec, int tnsec, int t1sec, int t1nsec){
        long double returnedValue;
        returnedValue = ((long double) (t1sec - tsec)) + ((long double)(t1nsec - tnsec))/1000000000;
        return returnedValue;
    }







    //velocita, tempo, frameid;

    float v_x;
    float v_y;
    float w_z;

    int t_k_sec;
    int t_k1_sec;
    int t_k_nsec;
    int t_k1_nsec;








    string frame_id;


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
