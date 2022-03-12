
## Getting started

1. `cd` to `experimental/packages/third-party/orbslam3`
2. `edit run.bash`
3. Change `WORKSPACE` to whatever the catkin_ws is that you want (the one with openvslam)
4. Make sure the Docker display args are correct
5. `./run.bash build` to build the container
6. `./run.bash orbslam3` to run orbslam3 ins tereo mode. Play a bag file with `/camera1`
