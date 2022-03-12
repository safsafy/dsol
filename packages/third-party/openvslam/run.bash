#!/bin/bash

export WORKSPACE=$HOME/workspaces/openvslam_ws

function docker_bash ()
{
    echo "$@"
    docker run -it --net=host --rm -v $WORKSPACE:/catkin_ws openvslam:noetic "$@"
}

function docker_build ()
{
    git submodule update --init openvslam_ros
    docker build -t openvslam:noetic . --build-arg NUM_THREADS=8 -f Dockerfile
    docker_bash catkin clean
    docker_bash catkin init
    docker_bash catkin build
}

function docker_vslam ()
{
    docker_bash rosrun openvslam_ros run_slam \
        -v /catkin_ws/src/experimental/packages/third-party/openvslam/orb_vocab.fbow \
        -c /catkin_ws/src/experimental/packages/third-party/openvslam/realsense_mono.yaml \
        "$@"
}

function docker_ ()
{
    echo "./run.bash build"
    echo "./run.bash bash [CMD ...]"
    echo "./run.bash vslam [args]"
}

CMD=$1; shift

docker_$CMD "$@"
