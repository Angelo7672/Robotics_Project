# Files inside the Project:

**cfg**: folder that contains the cfg file needed for the dynamic reconfiguration of the integration method, choosen between Euler’s and Runge-Kutta’s 
approximations.
  
**launch**: the launch file, my_project1.launch.
	
**msg**: the costum message for the rpm and the costum message for calibration.

**srv**: reset file for change pose (x,y,theta)
	
**src**: folder containing the 5 ROS nodes written in c++:

 	-kinematics.cpp: the node that evaluates forward kinematics for a 4 mecanum wheels robot. This node has an inizialization() method that takes the
 	 first registered ticks for each encoder, stores them in the kinematics class and send a header msg to odometry for initialize time. 
 	 The kinematics() method performs the actual forward kinematics, extracting the velocity on the x and y axis and the angular velocity of the robot
	 given the difference in ticks in each encoder between two consecutive messages from the topic. It then publishes the evaluated data on the
	 /cmd_vel the velocities. 
    
	-odometry.cpp: the node handles the odometry via either the Euler or Runge-Integration method. Either method is chosen by the user via the dynamic 
 	 reconfigure. It then publishes the evaluated data on the /odom the velocities. It takes the parameters for initialization of x_0, y_0 and theta_0
	 from the file launch.
    	
	-wheel_speeds.cpp: the node evaluates the speed in rpm of each wheel by elaborating the data obtained from topic /cmd_vel.
    
 	-client.cpp: the node that handles the “Reset pose” service. "usage: reset_client new_x new_y new_quat_x new_quat_y new_quat_z new_quat_w"
    
	-estimator_for_calibration.cpp: the node that handles the calibration of the Robot’s parameters (wheel radius (r), distance from the geometric
	 centre of the robot and the centre of the wheel both on X (l) and Y(w) axis, encoder resolution(n)). It checks the difference between the real rpm
	 and our calculated rmp, if the first measurement is greater with a 3% difference, it reduces N by 5% and increases R by 2%. Instead if our rpm are 
	 greater than the real ones with a 3% difference, it increases N by 5% and reduces R by 2%. Finally check the real theta with ours: if the real
	 theta is greater than ours with a difference of 3% it reduces l by 5% and w by 2%. If, on the other hand, our calculated theta is greater than the
	 real one with a difference of 3%, we increase l by 5% and w by 2%.

# ROS Parameters:

**KINEMATICS**:

	-U1 is the rad/s of the front left wheel;
	-U2 is the rad/s of the front right wheel;
	-U3 is the rad/s of the rear left wheel;
	-U4 is the rad/s of the rear right wheel;
	-v_x, v_y, are the velocities of the robot along the X and Y axis of the Global Frame of Reference;
	-w_z is the angular velocity of the robot along the z axis in the Global Frame of Reference;
	-first_pub is the publisher for the first message containing the time;
	-kinematics_pub and the publisher to send messages containing v_x v_y w_z;
	-calibration_sub receives information on the new calibrated parameters;
	
**ODOMETRY**:

	-x_k1 is the position along the X axis given by either Euler’s or Runge-Kutta approximation method;
	-y_k1 is the position along the Y axis given by either Euler’s or Runge-Kutta approximation method;
	-theta_k1 is the yaw given by the odometry formula’s;
	-first_sub receives the first message to set the time;
	-kinematics_sub receives messages containing velocities;
	-odometry_pub sends messages containing calculated positions;
	
**WHEEL_SPEEDS**:

	-rmp_fl are the rpm of the fl wheel;
	-rmp_fr are the rpm of the fr wheel;
	-rmp_rl are the rpm of the rl wheel;
	-rmp_rr are the rpm of the rr wheel;
	
**CLIENT**:

	-x y quat_x quat_y quat_z quat_w;
	
**ESTIMATOR_FOR_CALIBRATION**:

	l w N R;
	
**MY_PROJECT1**:

	x_0 y_0 quat_x quat_y quat_z quat_w, initial pose for every bag;

**CUSTOM MESSAGES AND SRV**:
The RPM custom messages structure is:

	Header header
	float64 rpm_fl;
	float64 rpm_fr;
	float64 rpm_rl;
	float64 rpm_rr;
	
The reset message structure is:

	float64 new_x
	float64 new_y
	float64 new_theta
	---
	float64 old_x
	float64 old_y
	float64 old_theta
	
The calibration message structure is:

	float64 r (wheel radius)
	float64 n (encoder resolution)
	float64 l (position along the x axis of the wheel's centre)
	float64 w (position along the y axis of the wheel's centre)
	
# DESCRIPTION ON HOW TO START/USE THE NODES:
The project can e started via

	roslaunch project1 myproject1.launch
	
The Reset service must be started via command line inputting 

	rosrun project1 reset_client new x the new y new_quat_x new_quat_y new_quat_z new_quat_w

# ABOUT CALIBRATION:
Our idea of ​​calibration (described above) is to run the bag first by adding the "calibrators" node:

	roslaunch project1 my_project1.launch
	rosrun project1 calibrator
	rosbag play bagi.bag
	
Once the bag is finished:

	rosrun project1 reset_client "just copy initial pose from my_project1.launch"
	
Calibrate the parameters before running each bag

# TF TREE STRUCTURE:
	world --> odom --> base_link
