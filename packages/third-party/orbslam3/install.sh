#!/bin/bash

BUILD_TYPE="Release"
INSTALL_PREFIX=$2
INSTALL_ARG=$3

install_dbow2 ()
{
    echo "Configuring and building Thirdparty/DBoW2 ..."
    pushd ORB_SLAM3/Thirdparty/DBoW2

    mkdir -p build
    pushd build
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    make -j

    popd
    popd
}


install_g2o ()
{
    echo "Configuring and building Thirdparty/g2o ..."
    pushd ORB_SLAM3/Thirdparty/g2o

    mkdir -p build
    pushd build
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    make -j

    popd
    popd
}

install_sophus ()
{
    echo "Configuring and building Thirdparty/Sophus ..."
    pushd ORB_SLAM3/Thirdparty/Sophus

    mkdir -p build
    pushd build
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    make -j

    popd
    popd
}

install_pangolin ()
{
  git clone https://github.com/stevenlovegrove/Pangolin.git

  pushd Pangolin

  git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81
  sed -i -e "193,198d" ./src/utils/file_utils.cpp
  mkdir -p build
  pushd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DBUILD_PANGOLIN_LIBREALSENSE=OFF \
    -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_PANGOLIN_PLEORA=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_TOON=OFF \
    -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
    -DBUILD_PANGOLIN_V4L=OFF \
    -DBUILD_PANGOLIN_VIDEO=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF \
    -DBUILD_PYPANGOLIN_MODULE=OFF \

  make -j $INSTALL_ARG

  popd
  popd
}


install_vocabulary ()
{
    echo "Uncompress vocabulary ..."

    pushd ORB_SLAM3/Vocabulary
    tar -xf ORBvoc.txt.tar.gz

    popd
}

install_orbslam3 ()
{
    echo "Configuring and building ORB_SLAM3 ..."
    pushd ORB_SLAM3

    mkdir -p build
    pushd build
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
    make -j4

    popd
    popd
}

install_ ()
{
    install_dbow2
    install_g2o
    install_sophus
    install_vocabulary
    install_orbslam3
}

install_$1
