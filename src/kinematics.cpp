#include "ros/ros.h"
#include "std_msgs/String.h"
#include "/Velocities.h"

#include <sstream>

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

int main(int argc, char **argv) {
    float u_1;
    float u_2;
    float u_3;
    float u_4;
    float v_x;
    float v_y;
    float w;

    ros::init(argc, argv, "kinematics");  //per inizializzare il nodo
    ros::NodeHandle n;    //per inizializzare il nodo

    ros::Publisher kinematics = n.advertise<::Velocities>("velocities", 1000);

    u_1 = * (1/60) * (1/GEAR_RATIO);
    u_2 = * (1/60) * (1/GEAR_RATIO);
    u_3 = * (1/60) * (1/GEAR_RATIO);
    u_4 = * (1/60) * (1/GEAR_RATIO);





    v_x = (WHEEL_RADIUS / 2) * (u_1 + u_2);
    v_y = (WHEEL_RADIUS / 2) * (u_2 - u_3);
    w = -(WHEEL_RADIUS / 2) * ((u_1 - u_3) / (WHEEL_POSITION_ALONG_Y + WHEEL_POSITION_ALONG_X));

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        std_msgs::String msg;

        std::stringstream ss;

        // generate  msg
        ::Velocities velocities_msg;
        velocities_msg.v_x = v_x;
        velocities_msg.v_y = v_y;
        velocities_msg.w = w;




        kinematics.publish(msg);

        ros::spinOnce();    //per dare il permesso a ROS, se c'e' qualcosa da fare prima di continuare con il main loop (da fare sempre se non c'e' il calcolo parallelo

        loop_rate.sleep();  //per mantenere la frequenza


    }

    return 0;

}

