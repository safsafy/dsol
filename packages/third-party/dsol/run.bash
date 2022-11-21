#!/bin/bash

set -e

# Path to the workspace where you want DSOL on your computer
export WORKSPACE=$HOME/dsol_ws
# Path to the directory on your computer where you are storing the rosbag to play.
export DATA=$HOME/data/8881/20220302_152750_archive/
# Name of the rosbag inside the data folder on your computer.
export INPUT_ROSBAG=20220302_152750_archive.bag 
# Path to the directory on your computer to store the results.
export RESULTS=$HOME/results/dsol
# Name of the rosbag to record results.
export OUTPUT_ROSBAG=dsol_20220302_152750_archive.bag 
# Name of the camera to use for odometry (camera1, camera2, ...)
export CAMERA=camera1

# The WORKDIR (entry directorty upon starting the container is /catkin_ws as set in Dockerfile)

function docker_bash ()
{
    echo "$@"

    docker run -it --net=host --rm \
        -v $WORKSPACE:/catkin_ws:rw \
        -v $DATA:/data:ro \
        -v $RESULTS:/results:rw \
        dsol_docker \
        "$@"
}

function docker_build ()
{
    docker build -f Dockerfile -t dsol_docker .

    mkdir -p $WORKSPACE/src 
    cd $WORKSPACE/src
    git clone https://github.com/versatran01/dsol.git

    docker_bash catkin init
    docker_bash catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
}

function docker_roscore ()
{
    docker_bash roscore
}

function docker_dsol_odom ()
{
    docker_bash bash -c "source /opt/ros/noetic/setup.bash \
                         && source /catkin_ws/devel/setup.bash \
                         && roslaunch dsol dsol_odom.launch camera:=$CAMERA"
}

function docker_record_output ()
{
    docker_bash rosbag record -O /results/$OUTPUT_ROSBAG \
                                /burro_base/odom \
                                /$CAMERA/gyro/sample \
                                /$CAMERA/infra1/camera_info \
                                /$CAMERA/infra1/image_rect_raw \
                                /$CAMERA/infra2/camera_info \
                                /$CAMERA/infra2/image_rect_raw \
                                /clock \
                                /dsol_odom/parray \
                                /dsol_odom/path_odom \
                                /dsol_odom/points \
                                /dsol_odom/pose_odom \
                                /odometer/state \
                                /tf \
                                /tf_static
}

function docker_play_data ()
{
    docker_bash rosbag play /data/$INPUT_ROSBAG
}

function docker_ ()
{
    echo "./run.bash build"
    echo "./run.bash bash [CMD ...]"
    echo "./run.bash orbslam3 [args]"
}

CMD=$1; shift

docker_$CMD "$@"
