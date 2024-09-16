#!/bin/zsh

PROJECT_DIR=${PWD}
cd ${PWD}/..
PROJECT_PARENT_DIR=${PWD}
echo ' ';
echo -e '\033[1;37mBuilding ThirdParty Libs\e[0m';
echo -e  '\033[1;37m=======================\e[0m';
echo ' ';

cd ${PROJECT_DIR}/Thirdparty/g2o;
if [ -d build ]; then
	rm -rf ./build ./lib
fi
mkdir build && cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j4;
echo ' ';
echo -e '\033[0;32m***Building g2o Done***\e[0m';
 echo ' ';

cd ${PROJECT_DIR}/Thirdparty/DBoW2;
if [ -d build ]; then
	rm -rf ./build ./lib
fi
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j4;
echo ' ';
echo -e '\033[0;32m***Building DBoW2 Done***\e[0m';
echo ' ';

cd ${PROJECT_DIR}/Data;
tar -xf ORBvoc.txt.tar.gz
echo -e '\033[0;32m***Vocab File Extraction Done***\e[0m';

cd ${PROJECT_DIR};
/bin/zsh /opt/ros/noetic/setup.zsh;
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PROJECT_PARENT_DIR};
echo -e '\033[0;32m***ROS_PACKAGE_PATH Updated***\e[0m';