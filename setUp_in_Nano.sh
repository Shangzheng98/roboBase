#!/bin/bash
#
# Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <Install Folder>"
    exit
fi
folder="$1"
user="nvidia"
passwd="nvidia"

echo "** Remove other OpenCV first"
sudo sudo apt-get purge *libopencv*

echo "** Install requirement"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y python2.7-dev python3.6-dev python-dev python-numpy python3-numpy
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2 v4l2ucp
sudo apt-get install -y curl
sudo apt-get install -y python-pip
sudo -H pip install -U jetson-stats
sudo apt -y install libboost-all-dev
sudo apt-get -y install vino
echo "** Install boost lib successfully"
sudo apt-get update            

echo "** Installing glog"
git clone https://github.com/google/glog.git
cd glog
cmake -H. -Bbuild -G "Unix Makefiles"
cmake --build build
sudo cmake --build build --target install
cd ..

echo "** Download opencv-4.1.1"
#cd $folder
curl -L https://github.com/opencv/opencv/archive/4.1.1.zip -o opencv-4.1.1.zip
curl -L https://github.com/opencv/opencv_contrib/archive/4.1.1.zip -o opencv_contrib-4.1.1.zip
unzip opencv-4.1.1.zip
unzip opencv_contrib-4.1.1.zip
cd opencv-4.1.1/

echo "** Apply patch"
sed -i 's/include <Eigen\/Core>/include <eigen3\/Eigen\/Core>/g' modules/core/include/opencv2/core/private.hpp

echo "** Building..."
mkdir release
cd release/
cmake -D WITH_CUDA=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.1.1/modules -D WITH_LIBV4L=ON -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j3
sudo make install
#echo 'export PYTHONPATH=$PYTHONPATH:'$PWD'/python_loader/' >> ~/.bashrc
#source ~/.bashrc
echo "** Install opencv-4.1.1 successfully"

git clone https://github.com/Tencent/rapidjson.git
cd rapidjson/
mkdir build
cd build
cmake ..
make
sudo make install
echo "** Install rapidjson successfully"
echo "** Bye :)"
