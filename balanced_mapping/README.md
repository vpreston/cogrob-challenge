# balanced-mapping

This package governs the balance between Active Slam and Science Mapping for the Grand Challenge.  This is intended to be the interface that will provide the final set of next optimal points to sample from, based on the ```meta_beta``` parametera that will be set in the launch file.

```meta_beta = 0``` corresponds to only doing Active SLAM only and not considering Science values at all

```meta_beta = 1``` corresponds to only doing Science Mapping only 

Inbetween 0 and 1, the focus shifts from valuing Active SLAM reward value to Science Mapping reward value linearly.  This is done by taking the top ```num_points``` (another parameter set in the launch file) points from each set (if ```meta_beta``` is not 0 or 1) and then re-calculating the combined relative reward function and ordering them accordingly (total max length of 2*```num_points```  (it all top points are different), but stopping after number of balanced points reaches ```num_points``` ). 

(**NOTE TO TEAM: this is deviation from what we previously discussed where we would just re-query the GP at the Active SLAM points.  The reason for this is that the previously discussed approach provides a strong bias to the Active SLAM points since we are only considering GP points over that small subset of the map).**

**Based on the taking the top ```num_points``` points from each of the Active SLAM And Science-Mapping nodes, it may be better to just have a single interface in the balanced mapping node to the rest of the teams and then we integrate the Active SLAM and Science Mapping code into a single Balanced Mapping node so we don't have to recompute everything?**

This code currently assumes that the Active SLAM and Science Mapping points are calculated from the same map at the same resolution (so a point at (1.45, -1.45), for example, exists in both maps and no interpolation is required).  

To run a simple simulation environment, please do the following:

```export TURTLEBOT3_MODEL=burger```

```roslaunch balanced_mapping balanced_mapping.launch```

An instance of RVIZ should appear, you may have to wait several seconds to let the movement and navigation server connect before goals will be processed (up to 40-60 seconds if using the VM).  To request new points use the command:

```rostopic pub -1 /semaphore std_msgs/Bool -- 'True'```

**To visualize the set of goal points in RVIZ, make sure to go to "Add", then "by topic" then "/possible_points"!**

* TODO: import final Science Mapping code
* TODO: efficient sampling from SLAM map value at GP points
* TODO: efficient sampling from GP map values at SLAM points
* TODO: Test ```meta_beta = 0``` (works and tested in RVIZ)  
* TODO: Test ```meta_beta = 1``` 
* TODO: Test ```meta_beta = 0.25, 0.5, 0.75``` (base works, but need to do efficient sampling)
* TODO: Test ```num_points= 3, 5, 10, 25, 50``` (works well with 10)
