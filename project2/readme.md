# Folders inside the Project:

**config**:

	- amcl.launch.xml -> contains amcl's configuration.
	- ekf_loc -> contains kalman's configuration.
	- gmapping.launch.xml -> contains gmapping's configuration.
	
**launch**:     

	- amcl.launch -> loads the map, adds odom tf, merges the two lasers, loads amcl's configuration, loads robot localization, starts trajectory_saver
	  node and finally opens Rviz.
	- gmapping.launch -> adds odom tf, merges the two lasers, loads gmapping's configuration, loads robot localization and finally opens Rviz.
	- localization.launch -> starts ekf_node.
	- scan_merger.launch -> uses ira_laser_tools pkg to merge front scan and rear scan to scan_multi, then uses tf to transform base_footprint
	  to base_link and base_link to scan. base_link is also transformed to laser_rear and front_rear.
		 
**map**: 	

	- map.pgm is the map's image.
	- map.yaml contains map's info.
	    
**rviz**: 	

	- loc.rviz is Rviz's configuration for localization.
	- map.rviz is Rviz's configuration for mapping.
	      
**scripts**:    

	- map_and_trajectory.py is a node that copies map in map folder and writes on it with OpneCv the robot's path, and finally saves the
	  trajectory in trajectory folder.
	
**src**: 	

	- odom_tf.cpp is a node that subscribes /odom and broadcasts it on base_footprint.
	
**trajectory**: 

	- trajectory.png is the robot's path written by trajectory_saver about current bag.
	- path_bag2.png is the robot's path about bag2.
	- path_bag3.png is the robot's path about bag3.
	  NB: path_bag2.png and path_bag3.png were made with trajectory_saver and then renamed.

# TF TREE:

	map --> odom --> base_footprint --> base_link --> laser_front		   
						       |
						       --> scan
						       |
						       --> laser_rear
						
# BAG:
	We have used bag1 for mapping and bag2 and bag3 for localization.		
	We have used gmapping for map creation.
	
# HOW TO USE:

	roslaunch project2 gmapping.launch
	rosrun map_server map_saver -f map (in map folder or amcl.launch won't work!)
	roslaunch project2 amcl.launch (trajectory_saver will start with its)
