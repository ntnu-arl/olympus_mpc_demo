# Setting up the controller

## Installation steps:
1.  The `mpc_controller` package requires [acados](https://docs.acados.org/index.html). To install it, run the following script:

``` bash
$ ./install_prerequisites.sh  [installation_path]
```
- If no installation path is provided, acados will be installed in `home/acados`. 

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
