#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"
#include "sensor_msgs/JointState"
#include "std_msgs/Header"
#include <cmath>
#include <string>

#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

class kinematics_tick{
private:
    int seq;
    string frame_id;
    int time_sec;
    int time_nsec;
    float front_left_ticks;
    float front_right_ticks;
    float rear_left_ticks;
    float rear_right_ticks;
public:
    bool first;

    kinematics_tick(){
        this->first = false;
    }

    void set_front_left(float ticks){ this->front_left_ticks = ticks; }
    void set_front_right(float ticks){ this->front_right_ticks = ticks; }
    void set_rear_left(float ticks){ this->rear_left_ticks = ticks; }
    void set_rear_right(float ticks){ this->rear_right_ticks = ticks; }
    void set_seq(int seq){ this->seq = seq; }
    void set_frame_id(string f_id){ this->frame_id = f_id; }
    void set_time_sec(int sec){ this->time_sec = sec; }
    void set_time_nsec(int nsec){ this->time_nsec=nsec; }

    float get_front_left_ticks(){ return this->front_left_ticks; }
    float get_front_right_ticks(){ return this->front_right_ticks; }
    float get_rear_left_ticks(){ return this->rear_left_ticks; }
    float get_rear_right_ticks(){ return this->rear_right_ticks; }
    int get_seq(){ return this->seq; }
    string get_frame_id(){ return this->frame_id; }
    int get_time_sec(){ return this->time_sec; }
    int get_time_nsec(){ return this->time_nsec; }
};

void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if(kinematics_tick::first) {
        kinematics(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->header.stamp.sec,msg->header.stamp.nsec, msg->header.frame_id, msg->header.seq);
    } else {    //durante il primo giro perche' per calcolare la velocita' ci servono due misure
        initialization(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->header.stamp.sec,msg->header.stamp.nsec, msg->header.frame_id, msg->header.seq);
    }
}

void initialization(float front_left_velocity, float front_right_velocity, float rear_left_velocity, float rear_right_velocity, int time_sec, int time_nsec, string frame_id, int seq){
    ros::Publisher first_pub = n.advertise<std_msgs::Header>("first", 1000);

    //il primo giro mando solo header per inizializzare classi
    // generate  msg
    std_msgs::Header first_msg;

    first_msg.seq = seq;
    first_msg.frame_id = frame_id;
    first_msg.stamp.sec = time_sec;
    first_msg.stamp.nsec = time_nsec;

    first_pub.publish(first_msg);

    //aggiornamento dati
    kinematics_tick::set_seq(seq);
    kinematics_tick::set_frame_id(frame_id);
    kinematics_tick::set_time_sec(time_sec);
    kinematics_tick::set_time_nsec(time_nsec);
    kinematics_tick::set_front_left(front_left_velocity);
    kinematics_tick::set_front_right(front_right_velocity);
    kinematics_tick::set_rear_left(rear_left_velocity);
    kinematics_tick::set_rear_right(rear_right_velocity);

    kinematics_tick::first = true;  //dopo il primo giro sara' sempre a true
}

void kinematics(float front_left_velocity, float front_right_velocity, float rear_left_velocity, float rear_right_velocity, int time_sec, int time_nsec, string frame_id, int seq){
    float u_1;
    float u_2;
    float u_3;
    float u_4;
    float v_x;
    float v_y;
    float w;

    ros::Publisher kinematics_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    u_1 = (front_left_velocity - kinematics_tick::getfront_left_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_2 = (front_right_velocity - kinematics_tick::getfront_right_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_3 = (rear_left_velocity - kinematics_tick::getrear_left_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_4 = (rear_right_velocity - kinematics_tick::getrear_right_ticks()) * (1 / 60) * (1 / GEAR_RATIO) * (1 / ENCODERS_RESOLUTION) * 2 * M_1_PI;

    //fare check con dati matrice video
    //forse servono degli if per queste formule
    v_x = (WHEEL_RADIUS / 2) * (u_1 + u_2);
    v_y = (WHEEL_RADIUS / 2) * (u_2 - u_3);
    w = - (WHEEL_RADIUS / 2) * ((u_1 - u_3) / (WHEEL_POSITION_ALONG_Y + WHEEL_POSITION_ALONG_X));

    //aggiornamento dati
    kinematics_tick::set_front_left(front_left_velocity);
    kinematics_tick::set_front_right(front_right_velocity);
    kinematics_tick::set_rear_left(rear_left_velocity);
    kinematics_tick::set_rear_right(rear_right_velocity);
    kinematics_tick::set_seq(seq);
    kinematics_tick::set_frame_id(frame_id);
    kinematics_tick::set_time_sec(time_sec);
    kinematics_tick::set_time_nsec(time_nsec);

    // generate  msg
    geometry_msgs::TwistStamped velocities_msg;

    velocities_msg.header.seq = seq;
    velocities_msg.header.frame_id = frame_id;  //frame_id lo cambiamo?
    velocities_msg.header.stamp.sec = time_sec;
    velocities_msg.header.stamp.nsec = time_nsec;

    velocities_msg.linear.x = v_x;
    velocities_msg.linear.y = v_y;
    velocities_msg.linear.z = 0;
    velocities_msg.angular.x = 0;
    velocities_msg.angular.y = 0;
    velocities_msg.angular.z = w;

    kinematics_pub.publish(velocities_msg);
}

int main(int argc, char **argv) {
    kinematics_tick my_kinematics;
    ros::init(argc, argv, "kinematics");  //per inizializzare il nodo
    ros::NodeHandle n;                    //per inizializzare il nodo

    ros::Subscriber kinematics_sub = n.subscribe("wheel_states", 1000, wheel_statesCallback);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}