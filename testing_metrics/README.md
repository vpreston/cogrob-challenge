# testing-metrics

This package will launch a simulation environment that can be used by our other nodes for development. In addition it provides routines which will adjust parameters in various nodes, calculate metrics, and return an analysis of performance.

To run a simple simulation environment, please do the following:

```export TURTLEBOT3_MODEL=burger```
```roslaunch testing_metrics sim_enviro.launch```

An instance of Gazebo and RVIZ should appear, and you will have the ability to tele-op the vehicle. 

TODO: Add an environment for the grand challenge
TODO: Add nodes which will spoof inputs for active SLAM and science mapping
TODO: Add launch files for routines to test everything
TODO: Add metrics nodes