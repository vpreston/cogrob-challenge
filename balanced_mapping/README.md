# balanced-mapping

This package governs the balance between Active Slam and Science Mapping for the Grand Challenge.  This is intended to be the interface that will provide the final set of next optimal points to sample from, based on the ```meta_beta``` parameter that will be set in the launch file.

```meta_beta = 0``` corresponds to only doing Active SLAM only and not considering Science values at all

```meta_beta = 1``` corresponds to only doing Science Mapping only 

Inbetween 0 and 1, the focus shifts from valuing Active SLAM reward value to Science Mapping reward value linearly.  This is done by taking the top ```num_points``` (another parameter set in the launch file) points from each set (if ```meta_beta``` is not 0 or 1) and then re-calculating the combined relative reward function and ordering them accordingly (total max length of 2 times ```num_points```  (if all top points are different), but stopping after number of balanced points reaches ```num_points```). 

The other parameters that are in the ```balanced_mapping.launch``` file are specific to either the Active SLAM implementation (wall_const, unknown_const, sigma, pixel_dist, pick_randomly) or the Science Mapping implementation (science_beta) are commented as such and the specific impact of those parameters on the algorithm can be found in their respective package readme files.

Current Assumptions:
 - SLAM map is in an occupancy grid from the /map topic (**verified with SLAM team**)
 - GP map will be based on an occupancy grid as well, so it will be the same frame as the /map topic (**verified with GP team**)

To run a simple simulation environment, please do the following:

```export TURTLEBOT3_MODEL=burger```

```roslaunch balanced_mapping balanced_mapping.launch```

An instance of RVIZ should appear, you may have to wait several seconds to let the movement and navigation server connect before goals will be processed (up to 40-60 seconds if using the VM, 10-20 seconds if running natively).  To request new points use the command:

```rostopic pub -1 /semaphore std_msgs/Bool -- 'True'```

**To visualize the set of goal points in RVIZ, make sure to go to "Add", then "by topic" then "/possible_points"!**  
