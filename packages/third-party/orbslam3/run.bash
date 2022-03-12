#!/bin/bash

set -e

export WORKSPACE=$HOME/workspaces/openvslam_ws

function docker_bash ()
{
    echo "$@"
    DISPLAY=':1'
    xhost +local:
    XAUTH=$HOME/.Xauthority

    docker run -it --net=host --rm \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $WORKSPACE:/catkin_ws orbslam3:noetic \
        "$@"
}

function docker_build ()
{
    git submodule update --init ORB_SLAM3

    # 20.04 / Noetic doesn't ship with OpenCV 4.4, so downgrade
    sed -i 's/OpenCV 4.4/OpenCV 4.2/g' ORB_SLAM3/CMakeLists.txt

    docker build -t orbslam3:noetic . --build-arg NUM_THREADS=8 -f Dockerfile
    docker_bash catkin clean
    docker_bash catkin init
    docker_bash catkin build orbslam3_ros
}

function docker_orbslam3 ()
{
    docker_bash rosrun orbslam3_ros OrbSlam3Stereo \
        /tmp/ORB_SLAM3/Vocabulary/ORBvoc.txt \
        /tmp/ORB_SLAM3/Examples_old/Stereo-Inertial/RealSense_D435i.yaml \
        false \
        /camera/left/image_raw:=/camera1/infra1/image_rect_raw \
        /camera/right/image_raw:=/camera1/infra2/image_rect_raw
}

function docker_ ()
{
    echo "./run.bash build"
    echo "./run.bash bash [CMD ...]"
    echo "./run.bash orbslam3 [args]"
}

CMD=$1; shift

docker_$CMD "$@"
