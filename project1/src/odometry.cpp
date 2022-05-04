#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "project1/Reset.h"
#include <cmath>

class odometry_class{
public:
    odometry_class(){
        this->resetService = this->n.advertiseService("reset", &odometry_class::reset_callback, this);
        this->odometry_pub = this->n.advertise<nav_msgs::Odometry>("odom", 1000);
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

    void odometry(ros::Time time, double v_x, double v_y, double w){
        double x_k1;
        double y_k1;
        double theta_k1;

        //Euler integration
        x_k1 = x_k + v_k * (time - last_time).toSec() * cos(theta_k);  //angoli in radianti
        y_k1 = y_k + v_k * (time - last_time).toSec() * sin(theta_k);
        theta_k1 = theta_k + w * (time - last_time).toSec();

        //aggiornamento dati
        last_time = time;
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
        odom_msg.pose.pose.orientation = transformStamped.transform.rotation;   //non sono sicuro
        //set the velocity
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.linear.y = v_y;
        odom_msg.twist.twist.angular.z = w;

        pub(odom_msg);
    }

    void first_Callback(const std_msgs::Header::ConstPtr& msg){ //inizializzazione tempo
        last_time = msg->stamp;
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

    ros::Time last_time;
    double x_k;
    double y_k;
    double theta_k;
};

int main(int argc, char **argv) {
    odometry_class my_odometry;
    ros::init(argc, argv, "odometry");  //per inizializzare il nodo
    ros::NodeHandle n;                  //per inizializzare il nodo

    //set initial pose
    //get parameter /initial_pose from parameter server
    double x_0;
    double y_0;
    double theta_0;
    n.getParam("/x_0", x_0);
    n.getParam("/y_0", y_0);
    n.getParam("/theta_0", theta_0);
    my_odometry.set_x_k(x_0);
    my_odometry.set_y_k(y_0);
    my_odometry.set_theta_k(theta_0);

    ros::Subscriber first_sub = n.subscribe("first", 1000, &odometry_class::first_Callback, &odometry_class);
    ros::Subscriber kinematics_sub = n.subscribe("cmd_vel", 1000, &odometry_class::cmd_velCallback, &odometry_class);

    ros::spin();      //solo ROS, e' piu' efficiente perche' non considera ulteriori funzioni

    return 0;
}

/*//Runge-Kutta
x_k1 = x_k + v_k * t_s * cos(theta_k + (w_k * t_s)/2);
y_k1 = y_k + v_k * t_s * sen(theta_k + (w_k * t_s)/2);
theta_k1 = theta_k + w_k * t_s;*/
