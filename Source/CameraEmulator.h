#pragma once

#include <string>
#include <opencv2/videoio.hpp>
#include "Frame.h"

using std::string;

class CameraEmulator {
public:
    CameraEmulator(string filename);
    ~CameraEmulator();

    double getWidth();
    double getHeight();
    double getVideoTime();    // milliseconds
    double getFrameTime();
    void acquireFrames(double time, Frame& out1, Frame& out2);
    void nextFrame(Frame& out);
    bool isFinished();

private:
    cv::VideoCapture mVideo;
};