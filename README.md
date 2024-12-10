# Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped

<div align="center">
  <img src="media/teaser.gif" alt="Teaser of the proposed controller." />
</div>

This repository contains the code for the paper ["Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped"](https://arxiv.org/abs/2409.14567).

An overview of the paper can be found [here](https://michalispapadakis.github.io/mpc_olympus/).

The main contents of this repository include:

* A simulation environment in drake and ROS1, with a simplified model of the olympus robot, to test the proposed attitude controller.
( The full model along with more details about the design and overall can be found in the  [Olympus design repo](https://github.com/ntnu-arl/Olympus-design).)
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

## References
If you use this work in your research, please cite the following publications:

**Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped** [Link to paper](https://arxiv.org/abs/2409.14567)
```bibtex
@article{papadakis2024olympusmpc,
  title={Modeling and In-flight Torso Attitude Stabilization of a Jumping Quadruped},
  author={Papadakis, Michail and Olsen, Jørgen Anker and Poulakakis, Ioannis and Alexis, Kostas},
  journal={arXiv preprint arXiv:2409.14567},
  year={2024},
  url={https://arxiv.org/abs/2409.14567}
}
```


**Design and experimental verification of a jumping legged robot for martian lava tube exploration** [Link to paper](https://ieeexplore.ieee.org/abstract/document/10406863)
```bibtex
@inproceedings{olsen2023design,
  title={Design and experimental verification of a jumping legged robot for martian lava tube exploration},
  author={Olsen, J{\o}rgen Anker and Alexis, Kostas},
  booktitle={2023 21st International Conference on Advanced Robotics (ICAR)},
  pages={452--459},
  year={2023},
  organization={IEEE}
}
```

## Contact 

You can contact us for any questions:
- [Michail Papadakis](mailto:michael.d.papadakis@gmail.com)
- [Jørgen Anker Olsen](mailto:jorgen.a.olsen@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
