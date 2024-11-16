#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using std::vector;

struct Frame {
    cv::Mat data;
    cv::Mat descriptors;
    vector<cv::KeyPoint> keypoints;
    double time;
};
