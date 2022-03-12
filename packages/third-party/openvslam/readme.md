
## Getting started

1. `cd` to `experimental/packages/third-party/openvslam`
2. `edit run.bash`
3. Change `WORKSPACE` to whatever the catkin_ws is that you want (the one with openvslam)
4. `./run.bash build` to build the container
5. `./run.bash vslam [rosrun args]` to run vslam, rosrun args can be used to remap topics etc.

