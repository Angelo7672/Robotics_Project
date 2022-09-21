# Files inside the Project:

cfg: folder that contains the cfg file needed for the dynamic reconfiguration of the integration method, choosen between Euler’s and Runge-Kutta’s 
approximations.
  
launch: the launch file, my_project1.launch.
	
msg: the costum message for the rpm and the costum message for calibration.

srv: reset file for change pose (x,y,theta)
	
src: folder containing the 5 ROS nodes written in c++:

 -kinematics.cpp: the node that evaluates forward kinematics for a 4 mecanum wheels robot. This node has an inizialization() method that takes the
 first registered ticks for each encoder, stores them in the kinematics class and send a header msg to odometry for initialize time. 
 The kinematics() method performs the actual forward kinematics, extracting the velocity on the x and y axis and the angular velocity of the robot given 
 the difference in ticks in each encoder between two consecutive messages from the topic. It then publishes the evaluated data on the /cmd_vel the 
 velocities. 
    
 -odometry.cpp: the node handles the odometry via either the Euler or Runge-Integration method. Either method is chosen by the user via the dynamic 
 reconfigure. It then publishes the evaluated data on the /odom the velocities. It takes the parameters for initialization of x_0, y_0 and theta_0 from 
 the file launch.
    
 -wheel_speeds.cpp: the node evaluates the speed in rpm of each wheel by elaborating the data obtained from topic /cmd_vel.
    
 -client.cpp: the node that handles the “Reset pose” service. "usage: reset_client new_x new_y new_quat_x new_quat_y new_quat_z new_quat_w"
    
 -estimator_for_calibration.cpp: the node that handles the calibration of the Robot’s parameters (wheel radius (r), distance from the geometric centre 
 of the robot and the center of the wheel both on X (l) and Y(w) axis, encoder resolution(n)). It checks the difference between the real rpm and our 
 calculated rmp, if the first measurement is greater with a 3% difference, it reduces N by 5% and increases R by 2%. Instead if our rpm are greater 
 than the real ones with a 3% difference, it increases N by 5% and reduces R by 2%. Finally check the real theta with ours: if the real theta is greater
 than ours with a difference of 3% it reduces l by 5% and w by 2%. If, on the other hand, our calculated theta is greater than the real one with a
 difference of 3%, we increase l by 5% and w by 2%.
