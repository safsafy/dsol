
## Getting started

This code allows us to run VTR3 with Stereo images from [the VTR3 repo](https://github.com/utiasASRL/vtr3/tree/v3.0.0) in a Docker container. At this stage we can run with rosbags provided by the authors. The plan is to get the system running with data from our own rosbags.

### Setup

The prerequisites are having Docker and the [Nvidia Container Toolkit](https://github.com/NVIDIA/nvidia-docker) installed. The latter is used because we nneed to run a container with GPU access. 

1. As per the [VTR3 installation instructions](https://github.com/utiasASRL/vtr3/wiki/Installation-Guide), create the following directories in your computer 

   ```
   export VTRROOT=~/vtr-asrl            # root directory of VTR3 
   export VTRSRC=${VTRROOT}/vtr3        # source code of VTR3 (from the git repo)
   export VTRDEPS=${VTRROOT}/deps       # system dependencies of VTR3
   export VTRVENV=${VTRROOT}/venv       # python dependencies of VTR3 (not used atm)
   export VTRDATA=${VTRROOT}/data       # datasets for VTR3
   export VTRTEMP=${VTRROOT}/temp       # temporary data directory for testing
   ```

2. Clone the VTR3 repo (the tag version is important)

   `cd ${VTRSRC}
git clone --branch v3.0.0 --depth 1 git@github.com:utiasASRL/vtr3.git .`

3. Initialize the submodules

   There is a mismatch to one of the submodules. We need to change the name of the branch and then checkout a new commit.

   Change the name of the branch in the `.gitmodules` file for the `steam` submodule from `ros2-master` to `master`. Then run 

   `git submodule update --init --remote`

   Finally run

   ```
   cd ${VTRSRC}/main/src/deps/steam
   git switch --detach c825521fcbfd26707d4f207e628a65e7acf59282
   ```

   Don't run submodule update again after checking out the new commit as that may move us forward to the latest commit again.

4. Change the [Nvidia GPU compute capability](https://developer.nvidia.com/cuda-gpus) in [gpusurf line 16](https://github.com/utiasASRL/vtr3/blob/aae397c80642b8bc3e6089342ede4e085db9dba3/main/src/deps/gpusurf/gpusurf/CMakeLists.txt#L16) based on the GPU model on your laptop (defaults to 7.5).

### Build the container

1. Copy the Dockerfile from this repo to the VTR3 repo as we have made some minor adjustments. The building of the container takes time as it installs a lot of dependencies.

   `cp /path/to/experimental/packages/third-party/vtr3/Dockerfile ${VTRSRC}`

2. Build the container by running 

   `./run.bash build`

3. Install the UI inside the container

   I was unable to include the commands for installing the UI in the `run.bash` script so these must be run manually after launching the container.

   ```
   ./run.bash run_container
   cd ${VTRSRC}/main
   VTRUI=${VTRSRC}/main/src/vtr_ui/vtr_ui/frontend/vtr-ui
   npm --prefix ${VTRUI} install ${VTRUI}
   npm --prefix ${VTRUI} run build
   ```

### Run the code


