#!/bin/bash

export PROJECT_DIR=${PWD}

echo "***Building ThirdParty Libs ***";

cd ${PROJECT_DIR}/Thirdparty/g2o && mkdir build && cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j4;
echo "***Building g2o Done***";

cd ${PROJECT_DIR}/Thirdparty/DBoW2 && mkdir build && cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j4;
echo "***Building DBoW2 Done***";

cd ${PROJECT_DIR}/Data;
tar -xf ORBvoc.txt.tar.gz

cd ${PROJECT_DIR}

/bin/zsh /opt/ros/noetic/setup.zsh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PWD}


