#pragma once

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "Utils/Stats.h"
#include "Frame.h"
#include <opencv2/calib3d.hpp>

using std::string;

class Tracker {
public:
    Tracker(cv::Ptr<cv::Feature2D> detector, cv::Ptr<cv::DescriptorMatcher> matcher);

    // methods for keypoints detection and matching
    void setReference(const Frame& reference);
    void setTarget(const Frame& target);
    void track();
    void reset();

    // results of the tracking computation
    cv::Mat getHomography() {return mHomography;}
    double getDistanceAvg() {return mDistanceAvg;}
    double getAngleAvg() {return mAngleAvg;}
    double getDistanceStd() {return mDistanceStd;}
    double getAngleStd() {return mAngleStd;}
    bool good();
    Stats& getStats() {return mStats;}

    // getters
    void setHeight(double height) {mHeight = height;}
    void setWidth(double width) {mWidth = width;}

private:
    Frame mReference;
    Frame mTarget;
    Stats mStats;
    cv::Ptr<cv::Feature2D> mDetector;
    cv::Ptr<cv::DescriptorMatcher> mMatcher;
    cv::Mat mHomography;
    double mDistanceAvg, mDistanceStd, mAngleAvg, mAngleStd;
    double mWidth, mHeight;
    bool mGood;

    const double MATCH_RATIO = 0.8;         // constant for Lowe ratio test
    const double RANSAC_THRESHOLD = 2.5;
    const int MAX_TILE_KPS = 200;           // max number of keypoints per tile (of the image)
    // constants for multi-threaded detection and computing
    static const int NUMX;     // tiles int the x axis
    static const int NUMY;     // tiles int the y axis
    static const int OVERLAP; // pixels overlapping between tiles

    double printTime(string msg, double time);  // used to print intermediate times for the same timer
    void computeTranslation(cv::Point2f k1, cv::Point2f k2, double& outDistance, double& outAngle);     // compute the translation between two Point2f
    void detectAndCompute(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);  // multi threaded keypoint detection and descriptor computing
};
