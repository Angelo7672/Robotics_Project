#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>
#include <geometry_msgs/TransformStamped.h>
#include "project1/Reset.h"
#include <cmath>

class odometry_class{
public:
    odometry_class(){
        this->resetService = this->n.advertiseService("reset", &odometry_class::reset_callback, this);
        this->odometry_pub = this->n.advertise<nav_msgs::Odometry>("odom", 1000);
        this->f = boost::bind(&odometry_class::dynamic_callback, this, _1, _2);
        this->server.setCallback(f);
        this->integration_method = 0;
    }

    bool reset_callback(project1::Reset::Request  &req,
                        project1::Reset::Response &res) {
        res.old_x = this->x_k;
        res.old_y = this->y_k;
        res.old_theta = this->theta_k;
        this->x_k = req.new_x;
        this->y_k = req.new_y;
        this->theta_k = req.new_theta;
        ROS_INFO("Request to reset x to %f - Responding with x: %f\nRequest to reset y to %f - Responding with y: %f\nRequest to reset theta to %f - Responding with theta: %f",
                 (double)req.new_x, (double)res.old_x,
                 (double)req.new_y, (double)res.old_y,
                 (double)req.new_theta, (double)res.old_theta);
        return true;
    }

    void dynamic_callback(project1::parametersConfig &config, uint32_t level){
        ROS_INFO("New integration method: %d - Level %d ", config.integration_method, level);
        this->integration_method = config.integration_method;
    }

    void odometry(ros::Time time, double v_x, double v_y, double w_z){
        double x_k1;
        double y_k1;
        double theta_k1;

        switch(integration_method) {
            case 0:
                //Euler integration
                x_k1 = x_k + (v_x * cos(theta_k) - v_y * sin(theta_k)) * (time - last_time).toSec();
                y_k1 = y_k + (v_x * sin(theta_k) + v_y * cos(theta_k)) * (time - last_time).toSec();
                theta_k1 = theta_k + w_z * (time - last_time).toSec();
                break;
            case 1:
                //Runge-Kutta integration
                x_k1 = x_k + (v_x * cos(theta_k + (w_z * (time - last_time).toSec()) / 2) - v_y * sin(theta_k + (w_z * (time - last_time).toSec()) / 2)) * (time - last_time).toSec();
                y_k1 = y_k + (v_x * sin(theta_k + (w_z * (time - last_time).toSec()) / 2) + v_y * cos(theta_k + (w_z * (time - last_time).toSec()) / 2)) * (time - last_time).toSec();
                theta_k1 = theta_k + w_z * (time - last_time).toSec();
                break;
        }

        //aggiornamento dati
        this->last_time = time;
        this->x_k = x_k1;
        this->y_k = y_k1;
        this->theta_k = theta_k1;

        // generate  msg
        geometry_msgs::TransformStamped transformStamped;
        // set header
        transformStamped.header.stamp = time;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        // set x,y
        transformStamped.transform.translation.x = x_k1;
        transformStamped.transform.translation.y = y_k1;
        transformStamped.transform.translation.z = 0.0;
        // set theta
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_k1);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        // send transform
        transformation(transformStamped);

        // generate  msg
        nav_msgs::Odometry odom_msg;
        //Header
        odom_msg.header.frame_id = "odom";
        odom_msg.header.stamp = time;
        //set the position
        odom_msg.pose.pose.position.x = x_k1;
        odom_msg.pose.pose.position.y = y_k1;
        odom_msg.pose.pose.position.z =  0.37;  //approssimato
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        //set the velocity
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.linear.y = v_y;
        odom_msg.twist.twist.angular.z = w_z;

        pub(odom_msg);
    }

    void first_Callback(const std_msgs::Header::ConstPtr& msg){ //inizializzazione tempo
        this->last_time = msg->stamp;
    }

    void cmd_velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        odometry(msg->header.stamp,msg->twist.linear.x,msg->twist.linear.y,msg->twist.angular.z);
    }

    void transformation(geometry_msgs::TransformStamped transformStamped){ br.sendTransform(transformStamped); }

    void pub(nav_msgs::Odometry odom_msg){ odometry_pub.publish(odom_msg); }

    void set_x_k(double x_k){ this->x_k = x_k; }
    void set_y_k(double y_k){ this->y_k = y_k; }
    void set_theta_k(double theta_k){ this->theta_k = theta_k; }

private:
    ros::NodeHandle n;
    ros::ServiceServer resetService;
    ros::Publisher odometry_pub;

    tf2_ros::TransformBroadcaster br;

    dynamic_reconfigure::Server<project1::parametersConfig> server;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
    int integration_method;    //per dynamic_reconfigure

    ros::Time last_time;
    double x_k;
    double y_k;
    double theta_k;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");  //per inizializzare il nodo
    ros::NodeHandle n;                  //per inizializzare il nodo

    odometry_class my_odometry;

    //set initial pose
    //get parameter from parameter server
    double x_0,y_0,theta_0;
    double quat_x,quat_y,quat_z,quat_w;
    double roll,pitch,yaw;
    n.getParam("/x_0", x_0);
    n.getParam("/y_0", y_0);
    n.getParam("/quat_x", quat_x);
    n.getParam("/quat_y", quat_y);
    n.getParam("/quat_z", quat_z);
    n.getParam("/quat_w", quat_w);

    tf2::Quaternion q(quat_x,quat_y,quat_z,quat_w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);

    my_odometry.set_x_k(x_0);
    my_odometry.set_y_k(y_0);
    my_odometry.set_theta_k(yaw);


    ros::Subscriber first_sub = n.subscribe("first", 1000, &odometry_class::first_Callback, &my_odometry);
    ros::Subscriber kinematics_sub = n.subscribe("cmd_vel", 1000, &odometry_class::cmd_velCallback, &my_odometry);

    ros::spin();      //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}