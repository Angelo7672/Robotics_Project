#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include "project1/Calibration.h"

#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

class kinematics_tick{
public:
    kinematics_tick(){
        this->first_pub = this->nh.advertise<std_msgs::Header>("first", 1000);
        this->kinematics_pub = this->nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
        this->first = true;

        this->r = 0.077/*WHEEL_RADIUS*/;
        this->n = 43/*ENCODERS_RESOLUTION*/;
        this->w = 0.17/*WHEEL_POSITION_ALONG_Y*/;
        this->l = 0.18/*WHEEL_POSITION_ALONG_X*/;
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

        this->first = false;  //dopo il primo giro sara' sempre a false
    }

    void kinematics(double front_left_velocity, double front_right_velocity, double rear_left_velocity, double rear_right_velocity, ros::Time new_time){
        double u_1;
        double u_2;
        double u_3;
        double u_4;
        double v_x;
        double v_y;
        double w_z;
        double pi = 3.1415926535897932384626433832795028841971693993751058209749445923;

        u_1 = ((front_left_velocity - front_left_ticks) / ((new_time - time).toSec())) * (double)(1.0 / GEAR_RATIO) * (double)(1.0 / n) * (2.0 * pi);
        u_2 = ((front_right_velocity - front_right_ticks) / ((new_time - time).toSec())) * (double)(1.0 / GEAR_RATIO) * (double )(1.0 / n) * (2.0 * pi);
        u_3 = ((rear_left_velocity - rear_left_ticks) / ((new_time - time).toSec())) * (double)(1.0 / GEAR_RATIO) * (double)(1.0 / n) * (2.0 * pi);
        u_4 = ((rear_right_velocity - rear_right_ticks) / ((new_time - time).toSec())) * (double)(1.0 / GEAR_RATIO) * (double)(1.0 / n) * (2.0 * pi);

        v_x = (u_1 + u_2 + u_3 + u_4) * (r / 4);
        v_y = (-u_1 + u_2 + u_3 - u_4) * (r / 4);
        w_z = ((-u_1 + u_2 - u_3 + u_4) * (r / 4)) / (4 * (w + l));

        //aggiornamento dati
        this->front_left_ticks = front_left_velocity;
        this->front_right_ticks = front_right_velocity;
        this->rear_left_ticks = rear_left_velocity;
        this->rear_right_ticks = rear_right_velocity;
        this->time = new_time;

        // generate  msg
        geometry_msgs::TwistStamped velocities_msg;

        velocities_msg.header.stamp = time;

        velocities_msg.twist.linear.x = v_x;
        velocities_msg.twist.linear.y = v_y;
        velocities_msg.twist.angular.z = w_z;

        pub(velocities_msg);
    }

    void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if(!first) {
            kinematics(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->header.stamp);
        } else {    //durante il primo giro perche' per calcolare la velocita' ci servono due misure
            initialization(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->header.stamp);
        }
    }

    void pub(geometry_msgs::TwistStamped velocities_msg){ kinematics_pub.publish(velocities_msg); }

    void calibrationCallback(const project1::Calibration::ConstPtr& msg){
        this->r = msg->r;
        this->n = msg->n;
        this->l = msg->l;
        this->w = msg->w;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher first_pub;
    ros::Publisher kinematics_pub;

    ros::Time time;
    double front_left_ticks;
    double front_right_ticks;
    double rear_left_ticks;
    double rear_right_ticks;

    double r;
    double n;
    double l;
    double w;

    bool first;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics");  //per inizializzare il nodo
    ros::NodeHandle n;                    //per inizializzare il nodo

    kinematics_tick my_kinematics;

    ros::Subscriber wheel_states_sub = n.subscribe("wheel_states", 1000, &kinematics_tick::wheel_statesCallback, &my_kinematics);
    ros::Subscriber calibration_sub = n.subscribe("calibration", 1000, &kinematics_tick::calibrationCallback, &my_kinematics);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}