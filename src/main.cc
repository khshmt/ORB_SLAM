/**
 * This file is part of ORB-SLAM.
 *
 * Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
 *
 * ORB-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

// STD
#include <fstream>
#include <iostream>
#include <thread>
// ROS
#include <ros/package.h>
#include <ros/ros.h>
// External Libs
#include <opencv2/core/core.hpp>
// ORB_SLAM
#include "Converter.h"
#include "FramePublisher.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPublisher.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ORB_SLAM");
    ros::start();

    std::cout << std::endl
              << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << std::endl
              << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl
              << "This is free software, and you are welcome to redistribute it" << std::endl
              << "under certain conditions. See LICENSE.txt.\n";

    if (argc != 3) {
        std::cerr << std::endl
             << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package "
                "directory)\n";
        ros::shutdown();
        return 1;
    }

    // Load Settings and Check
    std::string strSettingsFile = ros::package::getPath("ORB_SLAM") + "/" + argv[2];

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    // Load ORB Vocabulary
    std::string strVocFile = ros::package::getPath("ORB_SLAM") + "/" + argv[1];
    std::cout << std::endl << "Loading ORB Vocabulary. This could take a while.\n";

    ORB_SLAM::ORBVocabulary Vocabulary;
    bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);

    if (!bVocLoad) {
        std::cerr << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory.\n";
        std::cerr << "Failed to open at: " << strVocFile << std::endl;
        ros::shutdown();
        return 1;
    }
    std::cout << "Vocabulary loaded!\n\n";

    // Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    // Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    // Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    // Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    // Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
    std::thread trackingThread(&ORB_SLAM::Tracking::Run, &Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    // Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    std::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, &LocalMapper);

    // Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    std::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    // Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    // This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    ros::Rate r(fps);

    while (ros::ok()) {
        FramePub.Refresh();
        MapPub.Refresh();
        Tracker.CheckResetByPublishers();
        r.sleep();
    }

    // Save keyframe poses at the end of the execution
    std::ofstream f;

    auto vpKFs = World.GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM::KeyFrame::lId);

    std::cout << std::endl << "Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << std::endl;
    std::string strFile = ros::package::getPath("ORB_SLAM") + "/" + "KeyFrameTrajectory.txt";
    f.open(strFile.c_str());
    f << std::fixed;

    for(const auto pKF : vpKFs ) {
        if (pKF->isBad())
            continue;

        const auto& R = pKF->GetRotation().t(); 
        const auto& q = ORB_SLAM::Converter::toQuaternion(R);
        const auto& t = pKF->GetCameraCenter();
        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
          << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << '\n';
    }
    f.close();

    ros::shutdown();

    return 0;
}
