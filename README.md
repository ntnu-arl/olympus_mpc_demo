# Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped

![Teaser of the proposed controller.](https://github.com/ntnu-arl/olympus_mpc_demo/tree/main/media/teaser.gif)

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

2. Follow the instructions of the `olympus_simulation` and `mpc_controller` to install these packages, and then build the rest of the repository. 
