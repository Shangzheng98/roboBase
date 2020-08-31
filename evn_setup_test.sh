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

echo "** Download lateste opencv"
#cd $folder
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
unzip opencv*
unzip opencv_contrib*
cd opencv*

echo "** Apply patch"
sed -i 's/include <Eigen\/Core>/include <eigen3\/Eigen\/Core>/g' modules/core/include/opencv2/core/private.hpp

echo "** Building..."
mkdir release
cd release/
cmake -D WITH_CUDA=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib8/modules -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
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
