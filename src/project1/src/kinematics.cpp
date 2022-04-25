#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"
#include "sensor_msgs/JointState"

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    kinematics(msg->velocity[0],msg->velocity[1],msg->velocity[2],msg->velocity[3]);

}

void kinematics(float front_left_velocity, float front_right_velocity, float rear_left_velocity, float rear_right_velocity,){
    float u_1;
    float u_2;
    float u_3;
    float u_4;
    float v_x;
    float v_y;
    float w;

    ros::Publisher kinematics_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    u_1 = front_left_velocity * (1/60) * (1/GEAR_RATIO);
    u_2 = front_right_velocity * (1/60) * (1/GEAR_RATIO);
    u_3 = rear_left_velocity * (1/60) * (1/GEAR_RATIO);
    u_4 = rear_right_velocity * (1/60) * (1/GEAR_RATIO);


    v_x = (WHEEL_RADIUS / 2) * (u_1 + u_2);
    v_y = (WHEEL_RADIUS / 2) * (u_2 - u_3);
    w = -(WHEEL_RADIUS / 2) * ((u_1 - u_3) / (WHEEL_POSITION_ALONG_Y + WHEEL_POSITION_ALONG_X));

    ros::Rate loop_rate(10);    //10 Hz di frequenza

    while (ros::ok()) {

        // generate  msg
        geometry_msgs::TwistStamped velocities_msg;
        velocities_msg.linear.x = v_x;
        velocities_msg.linear.y = v_y;
        velocities_msg.linear.z = 0;
        velocities_msg.angular.x = 0;
        velocities_msg.angular.y = 0;
        velocities_msg.angular.z = w;

        //time and frame_id

        kinematics_pub.publish(velocities_msg);

        ros::spinOnce();    //per dare il permesso a ROS, se c'e' qualcosa da fare prima di continuare con il main loop (da fare sempre se non c'e' il calcolo parallelo
        loop_rate.sleep();  //per mantenere la frequenza
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "kinematics");  //per inizializzare il nodo
    ros::NodeHandle n;                    //per inizializzare il nodo

    ros::Subscriber kinematics_sub = n.subscribe("wheel_states", 1000, wheel_statesCallback);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}

