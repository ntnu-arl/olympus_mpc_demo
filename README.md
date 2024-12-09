# Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped

<div align="center">
  <img src="media/teaser.gif" alt="Teaser of the proposed controller." />
</div>

This repository contains the code for the paper ["Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped"](https://arxiv.org/abs/2409.14567).

An overview of the paper can be found [here](https://michalispapadakis.github.io/mpc_olympus/).

The main contents of this repository include:

* A simulation environment in drake and ROS1, with a simplified model of the robot, to test the proposed attitude controller
* The code for the proposed controller


## Installation Instructions

1. Clone the repository
```shell
mkdir -p mpc_ws/src
cd mpc_ws/src
git clone git@github.com:ntnu-arl/olympus_mpc_demo.git
```

2. Install the simulation: [olympus_simulation](olympus_simulation/README.md#setup)

3. Install the controller: [mpc_controller](mpc_controller/README.md#installation-steps)


## Run an example:

In seperate terminals (after sourcing):

```shell
roslaunch olympus_drake_ros sim.launch    #run simulation
roslaunch mpc_controller wbc.launch       #run attitude controller
roslaunch mpc_controller plot.launch      #view important plots for the MPC
rosrun    mpc_controller reference_pub.py #publish attitude reference
```
