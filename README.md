# cogrob-challenge
A repository of ROS packages for MIT 16.412 Grand Challenge 2018. The Grand Challenge consisted of several teams focused on a number of themes (adaptive sampling, multi-agent planning, image classification, semantic mapping, etc) in order to integrate ROS modules onto a fleet of turtlebots. These turtlebots need to simultaneously map the physical obstacles in the world, and generate an estimated "habitat" map of a visually observable quantity (images of various types of textures and colors). 

This repository is primarily focused on the development of the adaptive sampling aspect of this challenge. There are two adaptive sampling tasks: selection of candidate locations for active-SLAM and selection of candidate locations to take an observation for the "habitat" map (henceforth referred to as science-mapping). We have also worked on a combinatorial method which considers both utility with respect to active-SLAM and science goals.


## Contents
In this rospackage, you can find:
* active_SLAM: selects candidate points along a frontier relevant to performing SLAM exploration and loop closures
* science_mapping: selects candidate points from a classification-GP representation which balance explore-exploit traits
* balanced_mapping: weighs active-SLAM and science_mapping points to select candidate functions which perform simultaneous optimization
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

For the complete active-SLAM stakc, use ```sim_enviro.launch```. Please note! The active-SLAM stack waits for an external command before publishing points (this simulates the call feature employed by other teams). To spoof this, in a terminal you can type the following:

```rostopic pub -1 /semaphore std_msgs/Bool -- 'True'```

Further comments can be found in the launch files and source code written.


## Robot Functionality
TODO
