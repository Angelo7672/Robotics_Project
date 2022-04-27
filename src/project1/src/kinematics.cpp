#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"
#include "sensor_msgs/JointState"
#include <cmath>
#include <string>

#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 0.07
#define WHEEL_POSITION_ALONG_X 0.200
#define WHEEL_POSITION_ALONG_Y 0.169
#define GEAR_RATIO 5
#define ENCODERS_RESOLUTION 42

void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    //kinematics(msg->velocity[0],msg->velocity[1],msg->velocity[2],msg->velocity[3],msg->header.time.sec,msg->header.time.nsec);
    kinematics(msg->position[0],msg->position[1],msg->position[2],msg->position[3],msg->header.time.sec,msg->header.time.nsec);

}
class kinematics_tick{
public:
        kinematicsTick(){
            this->front_left_ticks;
            this->front_right_ticks;
            this->rear_left_ticks;
            this->rear_right_ticks;
        }

        void setfront_left(float ticks){
            this->front_left_ticks=ticks;
        }
        void setfront_right(float ticks){
            this->front_right_ticks=ticks;
        }

        void setrear_left(float ticks){
            this->rear_left_ticks=ticks;
        }


        void setrear_right(float ticks){
            this->rear_right_ticks=ticks;
        }

        float getfront_left_ticks(){
            return this->front_left_ticks;
        }
        float getfront_right_ticks(){
            return this->front_right_ticks;
        }
        float getrear_left_ticks(){
            return this->rear_left_ticks;
        }

        float getrear_right_ticks(){
            return this->rear_right_ticks;
        }


        string getframe_id(){
            return this->frame_id;
        }

        int gettime_sec(){
            return this->time_sec;
        }

        int gettime_nsec(){
            return this->time_nsec;
        }





        void setframe_id(string f_id){
            this->frame_id=f_id;
        }

        void settime_sec(int sec){
            this->time_sec=sec;
        }

        void settime_nsec(int nsec){
            this->time_nsec=nsec;
        }

        float front_left_ticks;
        float front_right_ticks;
        float rear_left_ticks;
        float rear_right_ticks;

        string frame_id;

        int time_sec;
        int time_nsec;
        string







}
void kinematics(float front_left_velocity, float front_right_velocity, float rear_left_velocity, int rear_right_velocity, int time){
    float u_1;
    float u_2;
    float u_3;
    float u_4;
    float v_x;
    float v_y;
    float w;

    ros::Publisher kinematics_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    u_1 = (front_left_velocity - kinematics_tick::getfront_left_ticks())* (1/60) * (1/GEAR_RATIO) * (1/ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_2 = (front_right_velocity - kinematics_tick::getfront_right_ticks() )* (1/60) * (1/GEAR_RATIO) * (1/ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_3 = (rear_left_velocity - kinematics_tick::getrear_left_ticks())* (1/60) * (1/GEAR_RATIO) * (1/ENCODERS_RESOLUTION) * 2 * M_1_PI;
    u_4 = (rear_right_velocity - kinematics_tick::getrear_right_ticks())* (1/60) * (1/GEAR_RATIO) * (1/ENCODERS_RESOLUTION) * 2 * M_1_PI;


    kinematics_tick::setfront_left(u_1);
    kinematics_tick::setfront_right(u_2);
    kinematics_tick::setrear_left(u_3);
    kinematics_tick::setrear_right(u_4);





    v_x = (WHEEL_RADIUS / 2) * (u_1 + u_2);
    v_y = (WHEEL_RADIUS / 2) * (u_2 - u_3);
    w = - (WHEEL_RADIUS / 2) * ((u_1 - u_3) / (WHEEL_POSITION_ALONG_Y + WHEEL_POSITION_ALONG_X));


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


    kinematics_tick mykinematics;
    mykinematics.settime_sec(0);
    mykinematics.settime_nsec(0);


    ros::Subscriber kinematics_sub = n.subscribe("wheel_states", 1000, wheel_statesCallback);

    ros::spin();     //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}

