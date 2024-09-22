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

#include "KeyFrameDatabase.h"

namespace ORB_SLAM {

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary& voc) : mpVoc(&voc) {
    mvInvertedFile.resize(voc.size());
}

void KeyFrameDatabase::add(KeyFrame* pKF) {
    std::unique_lock lock(mMutex);
    for (const auto& bow : pKF->mBowVec)
        mvInvertedFile[bow.first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame* pKF) {
    std::unique_lock lock(mMutex);
    // Erase elements in the Inverse File for the entry
    for (const auto& bow : pKF->mBowVec) {
        // List of keyframes that share the word
        auto& lKFs = mvInvertedFile[bow.first];

        if (auto it = std::find(std::begin(lKFs), std::end(lKFs), pKF); it != std::end(lKFs)) {
            lKFs.erase(it);
        }
    }
}

void KeyFrameDatabase::clear() {
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

std::vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
    std::set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    std::list<KeyFrame*> lKFsSharingWords;

    /* Search all keyframes that share a word with current keyframes, but
        Discard keyframes connected to the query keyframe */
    {
        std::unique_lock lock(mMutex);

        for (const auto& bow : pKF->mBowVec) {
            auto& lKFs = mvInvertedFile[bow.first];

            for (auto pKFi : lKFs) {
                if (pKFi->mnLoopQuery != pKF->mnId) {
                    pKFi->mnLoopWords = 0;
                    // exclude connected keyframes
                    if (spConnectedKeyFrames.find(pKFi) == std::end(spConnectedKeyFrames)) {
                        pKFi->mnLoopQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if (lKFsSharingWords.empty())
        return std::vector<KeyFrame*>();

    std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (auto pKFi : lKFsSharingWords) {
        if (pKFi->mnLoopWords > maxCommonWords)
            maxCommonWords = pKFi->mnLoopWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (auto pKFi : lKFsSharingWords) {
        if (pKFi->mnLoopWords > minCommonWords) {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if (si >= minScore)
                lScoreAndMatch.push_back(std::make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for (auto& [score, pKFi] : lScoreAndMatch) {
        std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
        float bestScore = score;
        float accScore = score;
        KeyFrame* pBestKF = pKFi;

        for (auto pNKF : vpNeighs) {
            if (pNKF->mnLoopQuery == pKF->mnId && pNKF->mnLoopWords > minCommonWords) {
                accScore += pNKF->mLoopScore;
                if (pNKF->mLoopScore > bestScore) {
                    pBestKF = pNKF;
                    bestScore = pNKF->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;

    std::set<KeyFrame*> spAlreadyAddedKF;
    std::vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for (auto& [accScore, pBestKF] : lAccScoreAndMatch) {
        if (accScore > minScoreToRetain) {
            if (!spAlreadyAddedKF.count(pBestKF)) {
                vpLoopCandidates.push_back(pBestKF);
                spAlreadyAddedKF.insert(pBestKF);
            }
        }
    }
    return vpLoopCandidates;
}

/*
    this function is called from Tracking::Relocalisation() function in case if localization LOST state
*/
std::vector<KeyFrame*> KeyFrameDatabase::DetectRelocalisationCandidates(Frame* F) {
    std::list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        std::unique_lock lock(mMutex);

        for (auto& bow : F->mBowVec) {
            auto& lKFs = mvInvertedFile[bow.first];

            for (auto& pKFi : lKFs) {
                if (pKFi->mnRelocQuery != F->mnId) {
                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if (lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (auto pKFi : lKFsSharingWords) {
        if (pKFi->mnRelocWords > maxCommonWords)
            maxCommonWords = pKFi->mnRelocWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (auto pKFi : lKFsSharingWords) {
        if (pKFi->mnRelocWords > minCommonWords) {
            nscores++;
            float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(std::make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (auto& [score, pKFi] : lScoreAndMatch) {
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = score;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for (auto pNKF : vpNeighs) {
            if (pNKF->mnRelocQuery != F->mnId)
                continue;

            accScore += pNKF->mRelocScore;
            if (pNKF->mRelocScore > bestScore) {
                pBestKF = pNKF;
                bestScore = pNKF->mRelocScore;
            }
        }
        lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;
    std::set<KeyFrame*> spAlreadyAddedKF;
    std::vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (auto& [score, pKFi] : lAccScoreAndMatch) {
        const float& si = score;
        if (si > minScoreToRetain) {
            if (!spAlreadyAddedKF.count(pKFi)) {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

}  // namespace ORB_SLAM
