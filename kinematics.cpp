#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics");  //per inizializzare il nodo
    ros::NodeHandle n;    //per inizializzare il nodo

    ros::Publisher kinematics = n.subscribe("", 1000, ...Callback);    //da dove arrivano i dati


    float u_1;
    float u_2;
    float u_3;
    float u_4;


    float v_x;
    float v_y;
    float w_z;

    v_x = (WHEEL_RADIUS / 2) * (u_1 + u_2);
    v_y = (WHEEL_RADIUS / 2) * (u_2 - u_3);
    w_z = -(WHEEL_RADIUS / 2) * ((u_1 - u_3) / (WHEEL_POSITION_ALONG_Y + WHEEL_POSITION_ALONG_X));

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        std_msgs::String msg;

        std::stringstream ss;

        ss << " v_x";
        msg.data = ss.str();

        ss << " v_y";
        msg.data = ss.str();

        ss << " w_x";
        msg.data = ss.str();


        kinematics.publish(msg);

        ros::spinOnce();    //per dare il permesso a ROS, se c'e' qualcosa da fare prima di continuare con il main loop (da fare sempre se non c'e' il calcolo parallelo

        loop_rate.sleep();  //per mantenere la frequenza


    }

    return 0;

}

void ...Callback(const std_msgs::String::ConstPtr& msg) {   //funzione per i dati

}