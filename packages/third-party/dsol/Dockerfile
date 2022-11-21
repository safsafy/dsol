# This docker file is based on the instructions in https://github.com/versatran01/dsol/blob/main/.github/workflows/build.yaml
FROM osrf/ros:noetic-desktop

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]
 
# Apt dependencies  
RUN apt-get update
RUN apt-get install -qy g++ libeigen3-dev git python3-catkin-tools
RUN rosdep update
# Running rosdep install took a very long time so I commented it out and the installation still works
#RUN rosdep install --from-paths . --ignore-src -y -r --as-root apt:false

RUN mkdir catkin_ws
RUN mkdir data
RUN mkdir results

# Install glog
RUN git clone --depth 1 --branch v0.6.0 https://github.com/google/glog.git \
  && cd glog \
  && cmake -S . -B build -G "Unix Makefiles" -DCMAKE_CXX_STANDARD=17 \
  && cmake --build build \
  && sudo cmake --build build --target install

# Install fmt
RUN git clone --depth 1 --branch 8.1.0 https://github.com/fmtlib/fmt.git \
 && cd fmt \
 && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DCMAKE_CXX_STANDARD=17 -DFMT_TEST=False \
 && sudo make install

# Install abseil
RUN git clone --depth 1 --branch 20220623.0 https://github.com/abseil/abseil-cpp.git \
 && cd abseil-cpp \
 && mkdir build && cd build \
 && cmake -DABSL_BUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. \
 && sudo cmake --build . --target install

# Install sophus
RUN git clone https://github.com/strasdat/Sophus.git \
 && cd Sophus \
 && mkdir build && cd build \
 && git checkout 785fef3 \
 && cmake -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=17 .. \
 && sudo make install

# Install google benchmark
RUN git clone --depth 1 --branch v1.6.2 https://github.com/google/benchmark.git \
 && cd benchmark \
 && cmake -E make_directory "build" \
 && cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBENCHMARK_ENABLE_GTEST_TESTS=OFF ../ \
 && sudo cmake --build "build" --config Release --target install

# Install xtl
RUN git clone https://github.com/xtensor-stack/xtl.git \
 && cd xtl \
 && mkdir build && cd build \
 && cmake .. \
 && sudo make install

# Install xtensor
RUN git clone https://github.com/xtensor-stack/xtensor.git \
 && cd xtensor \
 && mkdir build && cd build \
 && cmake .. \
 && sudo make install

# Set the working folder at startup
WORKDIR /catkin_ws
