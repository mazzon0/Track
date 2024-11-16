
#include <iostream>
#include <math.h>
#include <future>
#include <array>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Tracker.h"
#include "Utils/Timer.h"
#include "Utils/MatrixUtils.h"
#include "Utils/OutData.h"

using std::cout;
using std::endl;
using std::cerr;
using std::array;

const int Tracker::NUMX = 6;
const int Tracker::NUMY = 4;
const int Tracker::OVERLAP = 10;

Tracker::Tracker(cv::Ptr<cv::Feature2D> detector, cv::Ptr<cv::DescriptorMatcher> matcher)
    : mDetector(detector), mMatcher(matcher), mGood(true) {}

void Tracker::setReference(const Frame& reference) {
    Timer timer; timer.start();

    mReference = reference;    
    detectAndCompute(mReference.data, mReference.keypoints, mReference.descriptors);
    mStats.kp1 = mReference.keypoints.size();

    cout << "setReference: " << timer.get() << "ms" << endl;
}

void Tracker::setTarget(const Frame& target) {
    Timer timer; timer.start();

    mTarget = target;
    detectAndCompute(mTarget.data, mTarget.keypoints, mTarget.descriptors);
    mStats.kp2 = mTarget.keypoints.size();

    cout << "setTarget: " << timer.get() << "ms" << endl;
}

void Tracker::track() {
    Timer timer; timer.start();
    double lastTime=0;

    // keypoint matching
    vector<std::vector<cv::DMatch>> matches;
    mMatcher->knnMatch(mReference.descriptors, mTarget.descriptors, matches, 2);
    lastTime += printTime("track-matching", timer.get());

    // Lowe ratio test
    vector<cv::Point2f> p1, p2;
    for(const auto& match : matches) {
        if(match[0].distance < MATCH_RATIO * match[1].distance) {
            p1.push_back(mReference.keypoints[match[0].queryIdx].pt);
            p2.push_back(mTarget.keypoints[match[0].trainIdx].pt);
        }
    }
    mStats.matches = p1.size();
    lastTime += printTime("track-lowe", timer.get()-lastTime);

    if(p1.size() < 4) {
        cerr << "ERROR: not enough matches: " << p1.size() << endl;
        mGood = false;
        return;
    }

    // homography
    cv::Mat inlierMask;
    mHomography = cv::findHomography(p1, p2, cv::RANSAC, RANSAC_THRESHOLD, inlierMask);
    lastTime += printTime("track-homography", timer.get()-lastTime);

    // translation
    int nInliers = 0;
    double d, a;
    vector<double> distances;
    vector<double> angles;
    for(size_t i=0; i<p1.size(); ++i) {     // for every match, if it is an inlier, the translation is computed
        if(inlierMask.at<unsigned char>(i)) {
            computeTranslation(p1[i], p2[i], d, a);
            distances.push_back(d);
            angles.push_back(a);
            ++nInliers;
        }
    }

    computeStats(distances, mDistanceAvg, mDistanceStd);
    computeStats(angles, mAngleAvg, mAngleStd);

    mStats.inliers = nInliers;
    lastTime += printTime("track-translation", timer.get()-lastTime);

    printTime("track (total)", timer.get());
}

void Tracker::reset() {
    mReference.keypoints.clear();
    mReference.descriptors = cv::Mat(0, 64, CV_8UC1);
    mTarget.keypoints.clear();
    mTarget.descriptors = cv::Mat(0, 64, CV_8UC1);
}

double Tracker::printTime(string msg, double time) {
    cout << msg << ": " << time << "ms" << endl;
    return time;
}

void Tracker::computeTranslation(cv::Point2f k1, cv::Point2f k2, double& outDistance, double& outAngle) {
    double x1, x2, y1, y2;
    x1 = k1.x;
    y1 = k1.y;
    x2 = k2.x;
    y2 = k2.y;

    outDistance = std::sqrt(std::pow(x2-x1, 2) + std::pow(y2-y1, 2));
    outAngle = x2>x1 ? acos((x2-x1)/outDistance) : -acos((x2-x1)/outDistance);
}

bool Tracker::good() {
    bool res = mGood;
    mGood = true;
    return res;
}

void Tracker::detectAndCompute(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    array<vector<cv::KeyPoint>, NUMX*NUMY> allKeypoints;
    array<cv::Mat, NUMX*NUMY> allDescriptors;

    array<cv::Mat, NUMX*NUMY> tiles;
    array<std::future<void>, NUMX*NUMY> futures;
    for(int i=0; i<NUMX*NUMY; ++i) {
        futures[i] = std::async(std::launch::async, [this, &tiles, &allKeypoints, &allDescriptors, &image, i /*, NUMX, NUMY, OVERLAP*/](){
            // computing region od interest bounds
            int idX = i/NUMY;
            int idY = i%NUMY;
            int x = mWidth*idX/NUMX/2 + mWidth/4 - OVERLAP;
            int y = mHeight*idY/NUMY/2 + mHeight/4 - OVERLAP;
            int w = (mWidth/NUMX)/2+2*OVERLAP;
            int h = (mHeight/NUMY)/2+2*OVERLAP;
            cv::Rect roi(x, y, w, h);
            tiles[i] = image(roi);

            // keypoints detection
            mDetector->detect(tiles[i], allKeypoints[i]);
            if(allKeypoints[i].size() > MAX_TILE_KPS) {
                allKeypoints[i].resize(MAX_TILE_KPS);
            }

            // descriptor computing
            mDetector->compute(tiles[i], allKeypoints[i], allDescriptors[i]);

            // adjusting coordinates
            for(auto& kp : allKeypoints[i]) {
                kp.pt.x += roi.x;
                kp.pt.y += roi.y;
            }
        });
    }

    for(int i=0; i<NUMX*NUMY; ++i) {
        futures[i].wait();
    }

    // combining results
    array<cv::Mat, NUMX*NUMY+1> tmpMats;
    tmpMats[0] = cv::Mat(64, 0, CV_8UC1);
    for(int i=0; i<NUMX*NUMY; ++i) {
        keypoints.insert(keypoints.end(), allKeypoints[i].begin(), allKeypoints[i].end());
    }
    concMatrices(allDescriptors.data(), allDescriptors.size(), descriptors);
}
