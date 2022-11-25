#!/bin/bash

set -e

export VTRROOT=~/vtr-asrl       # root directory of VTR3 
export VTRSRC=${VTRROOT}/vtr3   # source code of VTR3 (from the git repo)
export VTRDEPS=${VTRROOT}/deps  # system dependencies of VTR3
export VTRVENV=${VTRROOT}/venv  # python dependencies of VTR3 (not used atm)
export VTRDATA=${VTRROOT}/data  # datasets for VTR3
export VTRTEMP=${VTRROOT}/temp  # temporary data directory for testing

# Pass a specific command to be run in the container before closing it upon completion.
function docker_bash ()
{
    echo "$@"

    sudo docker run -it --rm --name vtr3 \
        --privileged \
        --network=host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ${VTRROOT}:${VTRROOT}:rw \
        vtr3 \
        "$@"
}

# Run the container so that we can enter it to execute different commands.
function docker_run_container ()
{
    sudo docker run -it --rm --name vtr3 \
        --privileged \
        --network=host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ${VTRROOT}:${VTRROOT}:rw \
        vtr3
}

function docker_build ()
{
    cd ${VTRSRC}

    sudo docker build -t vtr3 \
        --build-arg USERID=$(id -u) \
        --build-arg GROUPID=$(id -g) \
        --build-arg USERNAME=$(whoami) \
        --build-arg HOMEDIR=${HOME} .

    # Build and install the VTR3 packages and web-based GUI
    docker_bash bash -c "source /opt/ros/galactic/setup.bash &&
                         cd ${VTRSRC}/robots/ros2 &&
                         colcon build --symlink-install &&
                         source ${VTRSRC}/robots/ros2/install/setup.bash &&
                         cd ${VTRSRC}/main &&
                         colcon build --symlink-install"
}

function docker_ ()
{
    echo "./run.bash build"
    echo "./run.bash bash [CMD ...]"
    echo "./run.bash run_container"
}

CMD=$1; shift

docker_$CMD "$@"
