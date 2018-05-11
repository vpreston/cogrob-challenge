# cogrob-challenge: adaptive sampling
A repository of ROS packages for MIT 16.412 Grand Challenge 2018. The Grand Challenge consisted of several teams focused on a number of themes (adaptive sampling, multi-agent planning, image classification, semantic mapping, etc) in order to integrate ROS modules onto a fleet of turtlebots. These turtlebots need to simultaneously map the physical obstacles in the world, and generate an estimated "habitat" map of a visually observable quantity (images of various types of textures and colors). 

This repository is primarily focused on the development of the adaptive sampling aspect of this challenge. There are two adaptive sampling tasks: selection of candidate locations for active-SLAM and selection of candidate locations to take an observation for the "habitat" map (henceforth referred to as science-mapping). We have also worked on a combinatorial method which considers both utility with respect to active-SLAM and science goals.


## Contents
In this rospackage, you can find:
* active_SLAM: selects candidate points along a frontier relevant to performing SLAM exploration and loop closures
* science_mapping: selects candidate points from a classification-GP representation which balance explore-exploit traits
* balanced_mapping: weighs active-SLAM and science_mapping points to select candidate functions which perform simultaneous optimization **This will be the only package used during the actual demonstrations, see "robot functionality" section at the end of this readme**
* testing_metrics: contains launch files and helper functions for simulation and robot operations

This repository can be run on a local machine in simulation as a completely stand-alone stack, or can be run as a node on a turtlebot (burger model test).


## Logistics
A VM for this course was provided, with the following environment:
* ROS Kinetic Kame
* Ubuntu 16.04
* Turtlebot3 (burger model was used primarily)
* Python 2.7

It may be additionally useful to have:
* [scikit-learn](http://scikit-learn.org/stable/install.html)
* [opencv2](https://pypi.org/project/opencv-python/)
* Python 3.+ available

As a quirk of the multi-team collaboration, an additional repository is necessary to run the science_mapping functionality. Please clone [this repo](https://github.com/Keyrat06/Gaussian_Processes_Sampling) into science_mapping/scripts. Move the science_mapping.py node into this folder. Now, you're ready to roll.

This entire repository is intended to be cloned into an existing catkin workspace (~/catkin_ws/src/).

To learn more about catkin, ROS packages, and more, we invite the user to check out various tutorials:
* [Catkin](http://wiki.ros.org/catkin/Tutorials)
* [ROS Packages](http://wiki.ros.org/Packages)
* [Writing a Publsher and Subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)


## Simulation Functionality
In testing_metrics, several launch files are provided which allow this node to be run completely in a simulation environment for testing active-SLAM functionality. Tele-op and pre-defined trajectory following are also provided as baseline comparison methods. 

To run tele-op mode in the simplified simulation environment of the grand challenge, you can run:

```roslaunch testing_metrics gc_teleop.launch```

For a pre-defined trajectory, use ```trajectory_follower.launch```

For the complete active-SLAM stack, use ```sim_enviro.launch```. To see the balanced mapping functionality, then run ```roslaunch balanced_mapping balanced_mapping_sim.launch``` (please go into the balanced mapping folder to see how to modify the parameters to alter the balancing behavior.

Please note! The active-SLAM node and balanced-mapping node both wait for an external command before publishing points (this simulates the call feature employed by other teams). To spoof this, in a terminal you can type the following:

```rostopic pub -1 /semaphore std_msgs/Bool -- 'True'```

Further comments can be found in the launch files and source code written.

## API Resolution and Documentation
In an effort to consolidate the API for the entire projects, we have attempted to collaboratively documented the API (via topics and message types) in this [Full API Document](https://docs.google.com/document/d/1qubpyjDwm_5SQB4DYVmUq-Q-syaA3oULnaxs8GWRLWc/edit?usp=sharing).

However, there are some open questions that may cause us to update our code before the actual grand challenge:
### OPEN QUESTIONS AND POTENTIAL CHANGES BEFORE 15 MAY
1. What frame will the GP science belief map be in?  (This question will actually be answered by the Image Classification team since they are taking the images in a given frame and then the GP is updating a separately threaded python file to produce a new static belief map.  Since we are evaluating points for Active SLAM based on the OccupancyGrid message in the /map topic, having the GP belief map be a OccupancyGrid based on the the same resolution (0.05m), length (TBD), and width (TBD) would be ideal).

2. How are we closing the loop for requesting new points? There are two sub-cases for this:
	- For the Multi-Agent 1 and Underactuated team:  (Resolved: They will ask for new points via the /semaphore std_msgs/Bool)
	- For the Multi-Agent 2 team: We are not sure if they will want a constant update of PointCloud data or use the same method.

## Robot Functionality
The balanced_mapping package has the node scripts and launch files for all the functionality needed for the Grand Challenge.  The descriptions for each scenario are based off of the scenarios described in the [Grand Challenge Scenario Draft document](https://docs.google.com/document/d/1DEXqbQV11xOCvY5ATHI9VJYikEybKzSHXLU2wob9oqg/edit?usp=sharing). 

1. For the Minimal Scenario, the node should be brought up as follows:
 - When the first robot is sent in to generate the SLAM map, set the ```meta_beta``` parameter in ```balanced_mapping_only.launch``` to 0.0.  This will cause the robot to only run the Active SLAM code and ignore the science map altogether.  After this is set, then just run ```roslaunch balanced_mapping balanced_mapping_only.launch```
 - After the first robot is done and the map is created, then shutdown the node.
 - Next, change the ```meta_beta``` parameter to 1.0.  This will cause the next two robots to only run the Science Mapping code and ignore the Active SLAM points.  After this is set up, bring up the node again with ```roslaunch balanced_mapping balanced_mapping_only.launch```.
2. For the combined scenario with three turtlebots running simultaneously, the node should be brought up as follows:
 - Change the ```meta_beta``` parameter a value of 0.5. This will equally balance Active SLAM points and Science Points throughout the scenario.  ** Note:  We may update the balanced_mapping node to allow force a time-based increase in the meta_beta over time, which will force the turtlebots to value the Science Points more and more over time **
 - Run ```roslaunch balanced_mapping balanced_mapping_only.launch```


