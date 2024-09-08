#!/bin/bash

export PROJECT_DIR=${PWD}

echo "***Building ThirdParty Libs ***";

cd ${PROJECT_DIR}/Thirdparty/g2o;
if [ -d build ]; then
	rm -rf ./build ./lib
fi
mkdir build && cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j4;
echo "***Building g2o Done***";
 
cd ${PROJECT_DIR}/Thirdparty/DBoW2;
if [ -d build ]; then
	rm -rf ./build ./lib
fi
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j4;
echo "***Building DBoW2 Done***";

cd ${PROJECT_DIR}/Data;
tar -xf ORBvoc.txt.tar.gz

cd ${PROJECT_DIR}

if [ -n "$ZSH_VERSION" ]; then
	/bin/zsh /opt/ros/noetic/setup.zsh
elif [ -n "$BASH_VERSION" ]; then
	source /opt/ros/noetic/setup.bash
else
	echo "Unable to source ROS"
fi
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PWD}


