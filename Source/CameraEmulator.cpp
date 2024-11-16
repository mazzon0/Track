#include "CameraEmulator.h"

#include <iostream>
#include <algorithm>

using std::cout;
using std::cerr;
using std::endl;

CameraEmulator::CameraEmulator(string filename) {
    mVideo.open(filename);
    if(!mVideo.isOpened()) {
        cerr << "ERROR: failed to open the file " << filename << endl;
        return;
    }
}

CameraEmulator::~CameraEmulator() {
    mVideo.release();
}

double CameraEmulator::getWidth() {
    return mVideo.get(cv::CAP_PROP_FRAME_WIDTH);
}

double CameraEmulator::getHeight() {
    return mVideo.get(cv::CAP_PROP_FRAME_HEIGHT);
}

double CameraEmulator::getVideoTime() {
    double frameTime = 1000 / mVideo.get(cv::CAP_PROP_FPS); // frame time in milliseconds
    return mVideo.get(cv::CAP_PROP_FRAME_COUNT) * frameTime;
}

double CameraEmulator::getFrameTime() {
    return 1 / mVideo.get(cv::CAP_PROP_FPS);
}

void CameraEmulator::acquireFrames(double time, Frame& out1, Frame& out2) {
    double frameTime = 1000 / mVideo.get(cv::CAP_PROP_FPS); // frame time in milliseconds
    uint32_t frameID = (uint32_t)(time/frameTime);

    if(frameID != 0) {
        mVideo.set(cv::CAP_PROP_POS_FRAMES, (double)(frameID-1));

        cout << "Acquired frame time: " << mVideo.get(cv::CAP_PROP_POS_FRAMES)/mVideo.get(cv::CAP_PROP_FPS) << endl;

        mVideo >> out1.data;
        out1.data = out1.data.clone();
        out1.time = frameTime * (frameID-1);

        mVideo.set(cv::CAP_PROP_POS_FRAMES, (double)(frameID));
        mVideo >> out2.data;
        out2.time = frameTime * frameID;
    }
    else {  // for the first frame both reference and target are the same image
        cout << "Acquired frame time: " << mVideo.get(cv::CAP_PROP_POS_FRAMES)/mVideo.get(cv::CAP_PROP_FPS) << endl;
        
        mVideo >> out1.data;
        out1.time = 0;

        out2.data = out1.data;
        out2.data = 0;
    }
}

void CameraEmulator::nextFrame(Frame& out) {
    mVideo >> out.data;
}

bool CameraEmulator::isFinished() {
    return mVideo.get(cv::CAP_PROP_POS_FRAMES) == mVideo.get(cv::CAP_PROP_FRAME_COUNT);
}
