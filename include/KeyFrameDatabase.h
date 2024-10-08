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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

// STD
#include <list>
#include <mutex>
#include <set>
#include <thread>
#include <vector>
// ORB_SLAM
#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"
// ThirdParty
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

namespace ORB_SLAM {

class KeyFrame;
class Frame;

class KeyFrameDatabase {
   public:
    KeyFrameDatabase(const ORBVocabulary& voc);

    void add(KeyFrame* pKF);

    void erase(KeyFrame* pKF);

    void clear();

    // Loop Detection
    std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);

    // Relocalisation
    std::vector<KeyFrame*> DetectRelocalisationCandidates(Frame* F);

   protected:
    // Associated vocabulary
    const ORBVocabulary* mpVoc;

    // Inverted file
    std::vector<list<KeyFrame*>> mvInvertedFile;

    // Mutex
    std::mutex mMutex;
};

}  //namespace ORB_SLAM

#endif
