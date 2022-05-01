#include "ros/ros.h"
#include "geometry_msgs/TwistStamped"
#include "std_msgs/Header"
#include "nav_msgs/Odometry"
#include "project1/Reset.h"
#include <string>
#include <cmath>


class odometry_class(){
private:
    ros::ServiceServer resetService;

    int seq;
    string frame_id;
    int time_sec;
    int time_nsec;
    float x_k;
    float y_k;
    float theta_k;
public:
    odometry_class(){
        this-> resetService = this->n.advertiseService("reset", &odometry_class::reset_callback, this);
    }

    long double evaluate_Ts(int tsec, int tnsec){
        long double returnedValue;
        returnedValue = ((long double) (tsec - time_sec)) + ((long double)(tnsec - time_nsec))/1000000000;
        return returnedValue;
    }

    bool reset_callback(project1::Reset::Request  &req,
                        project1::Reset::Response &res) {
        res.old_x = this->x;
        res.old_y = this->y;
        res.old_theta = this->theta;
        this->x = req.new_x;
        this->y = req.new_y;
        this->theta = req.new_theta;
        ROS_INFO("Request to reset x to %f - Responding with x: %f\nRequest to reset y to %f - Responding with y: %f\nRequest to reset theta to %f - Responding with theta: %f",
                 (float)req.new_x, (float )res.old_x,
                 (float)req.new_y, (float )res.old_y,
                 (float)req.new_theta, (float )res.old_theta);
        return true;
    }

    void set_seq(int seq){ this->seq = seq; }
    void set_frame_id(string f_id){ this->frame_id = f_id; }
    void set_time_sec(int sec){ this->time_sec = sec; }
    void set_time_nsec(int nsec){ this->time_nsec=nsec; }
    void set_x_k(float x_k){ this->x_k = x_k; }
    void set_y_k(float y_k){ this->y_k = y_k; }
    void set_theta_k(float theta_k){ this->theta_k = theta_k; }

    int get_seq(){ return this->seq; }
    string get_frame_id(){ return this->frame_id; }
    int get_time_sec(){ return this->time_sec; }
    int get_time_nsec(){ return this->time_nsec; }
    float get_x_k(){ return this->x_k; }
    float get_y_k(){ return this->y_k; }
    float get_theta_k(){ return this->theta_k; }
};

void first_Callback(const std_msgs::Header::ConstPtr& msg){ //inizializzazione classe odometry
    odometry_class::set_seq(msg->seq);
    odometry_class::set_frame_id(msg->frame_id);
    odometry_class::set_time_sec(msg->stamp.sec);
    odometry_class::set_time_nsec(msg->stamp.nsec);
}

void cmd_velCallback(const geometry_msgs::TwistedStamped::ConstPtr& msg) {
    odometry(msg->header.seq,msg->header.frame_id,msg->header.stamp.sec,msg->header.stamp.nsec,msg->linear.x,msg->linear.y,msg->angular.z);
}

void odometry(int seq, string frame_id, int tsec, int tnsec, float v_x, float v_y, float w){
    float x_k1;
    float y_k1;
    float theta_k1;

    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

    //Euler integration
    x_k1 = odometry_class::get_x_k + v_k * odometry_class::evaluate_Ts(tsec,tnsec) * cos(theta_k);  //angoli in radianti
    y_k1 = odometry_class::get_y_k + v_k * odometry_class::evaluate_Ts(tsec,tnsec) * sen(theta_k);
    theta_k1 = odometry_class::get_theta_k + w * odometry_class::evaluate_Ts(tsec,tnsec);

    odometry_class::set_seq(seq);
    odometry_class::set_frame_id(frame_id);
    odometry_class::set_time_sec(tsec);
    odometry_class::set_time_nsec(tnsec);
    odometry_class::set_x_k(x_k1);
    odometry_class::set_y_k(y_k1);
    odomedry_class::set_theta_k(theta_k1);

    // generate  msg
    nav_msgs::Odometry odom_msg;
    //Header
    odom_msg.header.seq = seq;
    odom_msg.header.frame_id = frame_id;    //frame_id lo cambiamo?
    odom_msg.header.stamp.sec = time_sec;
    odom_msg.header.stamp.nsec = time_nsec;

    odom_msg.child_frame_id = "";
    //PoseWithCovariance
    odom_msg.pose.pose.position.x = x_k1;
    odom_msg.pose.pose.position.y = y_k1;
    odom_msg.pose.pose.position.z =  ;
    odom_msg.pose.pose.orientation.x = ;
    odom_msg.pose.pose.orientation.y = ;
    odom_msg.pose.pose.orientation.z = ;
    odom_msg.pose.pose.orientation.w = ;
    for(int i = 0; i < 37; i++)
        odom_msg.pose.covariance[i] = 0;
    //TwistWithCovariance
    odom_msg.twist.twist.linear.x = v_x;
    odom_msg.twist.twist.linear.y = v_y;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = w;
    for(int i = 0; i < 37; i++)
        odom_msg.twist.covariance[i] = 0;

    odometry_pub.publish(odom_msg);
}

int main(int argc, char **argv) {
    odometry_class my_odometry;
    ros::init(argc, argv, "odometry");  //per inizializzare il nodo
    ros::NodeHandle n;                  //per inizializzare il nodo

    ros::Subscriber odometry_sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);
    ros::Subscriber first_sub = n.subscribe("first", 1000, first_Callback);

    ros::spin();      //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}

/*//Runge-Kutta
x_k1 = x_k + v_k * t_s * cos(theta_k + (w_k * t_s)/2);
y_k1 = y_k + v_k * t_s * sen(theta_k + (w_k * t_s)/2);
theta_k1 = theta_k + w_k * t_s;*/
