
## Getting started

This code allows us to run DSOL from https://github.com/versatran01/dsol in a Docker container.

### Setup

The only prerequisite is having docker installed. The DSOL code will be cloned during building.

1. `cd` to `experimental/packages/third-party/dsol`
2. Edit `run.bash` to have the directories at the top of the file point to the correct ones on your computer. Also specify the name of the camera to use (camera1, camera2, ...).

### Build the container

`./run.bash build` to build the container.

### Run the code

This container does not run with options for visualization so results must be written out to your computer and viewed in Rviz afterwards. DSOL has an Rviz config file under `dsol/launch/dsol_odom.rviz`, which can be found under the workspace folder you specified in `run.bash`.

Open four terminal windows and run the following commands in the separate windows in the order given below:
1. `./run.bash roscore`.
2. `./run.bash dsol_odom` to launch dsol.
3. `./run.bash record_output`  to record the results in a rosbag.
4. `./run.bash play_data` to play the input data from a rosbag.
