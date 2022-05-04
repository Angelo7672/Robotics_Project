#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <cmath>
#include <string>

#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

class kinematics_tick{
public:
    kinematics_tick(){
        this->first_pub = this->n.advertise<std_msgs::Header>("first", 1000);
        this->kinematics_pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
        this->first = false;
    }

    void initialization(double front_left, double front_right, double rear_left, double rear_right, ros::Time time){
        //il primo giro mando solo header per inizializzare classi
        // generate  msg
        std_msgs::Header first_msg;
        first_msg.stamp = time;
        first_pub.publish(first_msg);

        //aggiornamento dati
        this->time = time;
        this->front_left_ticks = front_left;
        this->front_right_ticks = front_right;
        this->rear_left_ticks = rear_left;
        this->rear_right_ticks = rear_right;

        this->first = true;  //dopo il primo giro sara' sempre a true
    }

    void pub(geometry_msgs::TwistStamped velocities_msg){
        kinematics_pub.publish(velocities_msg);
    }

    void set_front_left(float ticks){ this->front_left_ticks = ticks; }
    void set_front_right(float ticks){ this->front_right_ticks = ticks; }
    void set_rear_left(float ticks){ this->rear_left_ticks = ticks; }
    void set_rear_right(float ticks){ this->rear_right_ticks = ticks; }
    void set_time(ros::Time time){ this->time = time; }

    float get_front_left_ticks(){ return this->front_left_ticks; }
    float get_front_right_ticks(){ return this->front_right_ticks; }
    float get_rear_left_ticks(){ return this->rear_left_ticks; }
    float get_rear_right_ticks(){ return this->rear_right_ticks; }
    ros::Time get_time(){ return this->time; }
    bool get_first(){ return this->first; }

private:
    ros::NodeHandle n;
    ros::Publisher first_pub;
    ros::Publisher kinematics_pub;

    ros::Time time;
    float front_left_ticks;
    float front_right_ticks;
    float rear_left_ticks;
    float rear_right_ticks;

    bool first;
};

void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if(my_kinematics::get_first()) {
        kinematics(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->header.stamp);
    } else {    //durante il primo giro perche' per calcolare la velocita' ci servono due misure
        initialization(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->header.stamp);
    }
}

void kinematics(double front_left_velocity, double front_right_velocity, double rear_left_velocity, double rear_right_velocity, ros::Time time){
    double u_1;
    double u_2;
    double u_3;
    double u_4;
    double v_x;
    double v_y;
    double w;

    u_1 = (front_left_velocity - my_kinematics::get_front_left_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_2 = (front_right_velocity - my_kinematics::get_front_right_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_3 = (rear_left_velocity - my_kinematics::get_rear_left_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_4 = (rear_right_velocity - my_kinematics::get_rear_right_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;

    //fare check con dati matrice video
    //forse servono degli if per queste formule
    v_x = (WHEEL_RADIUS / 2) * (u_1 + u_2);
    v_y = (WHEEL_RADIUS / 2) * (u_2 - u_3);
    w = - (WHEEL_RADIUS / 2) * ((u_1 - u_3) / (WHEEL_POSITION_ALONG_Y + WHEEL_POSITION_ALONG_X));

    //aggiornamento dati
    my_kinematics::set_front_left(front_left_velocity);
    my_kinematics::set_front_right(front_right_velocity);
    my_kinematics::set_rear_left(rear_left_velocity);
    my_kinematics::set_rear_right(rear_right_velocity);
    my_kinematics::set_time_sec(time_sec);
    my_kinematics::set_time_nsec(time_nsec);

    // generate  msg
    geometry_msgs::TwistStamped velocities_msg;

    velocities_msg.header.stamp = time;

    velocities_msg.linear.x = v_x;
    velocities_msg.linear.y = v_y;
    velocities_msg.linear.z = 0;
    velocities_msg.angular.x = 0;
    velocities_msg.angular.y = 0;
    velocities_msg.angular.z = w;

    my_kinematics::pub(velocities_msg);
}

int main(int argc, char **argv) {
    kinematics_tick my_kinematics;
    ros::init(argc, argv, "kinematics");  //per inizializzare il nodo
    ros::NodeHandle n;                    //per inizializzare il nodo

    ros::Subscriber kinematics_sub = n.subscribe("wheel_states", 1000, wheel_statesCallback);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}