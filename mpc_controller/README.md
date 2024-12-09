# Setting up the controller

## Installation steps:
1.  The `mpc_controller` package requires [acados](https://docs.acados.org/index.html). To install it, run the following script:

``` bash
$ ./install_prerequisites.sh  [installation_path]
```
- It is recommended to not alter the default installation path, which is `home/acados`. 
- If an installation path is provided, the CMake and Make files in the `c_generated_code` folders must be changed accordingly.

2. If an installation path is provided, update the `ACADOS_INSTALL_DIR` in the `launch/wbc.launch`.

3. Build the package
``` bash
$ catkin build mpc_controller
```

4. To check that acados runs as expected, after sourcing the ws run:
```
roslaunch mpc_controller test.launch
```

The node should solve an MPC problem for one iteration and output the solution results.

## Run the whole body controller:

After sourcing the ws
```
roslaunch mpc_controller wbc.launch
```

## Run different scenarios:
The simulated scenarios can be changed by tweaking the config file [controller_params.yaml](config/controller_params.yaml)

- `controller_mode` changes the degree of freedom for the reorientation maneuver. It must be the same as the `base_joint_type` in  [olympus_param.yaml](../olympus_simulation/olympus_drake/config/olympus_param.yaml).
- `stabilization_mode` If set to `true`, the attitude controller tracks reference orientations. Otherwise, it tracks the torque specified at `manoeuvre_target_torque`