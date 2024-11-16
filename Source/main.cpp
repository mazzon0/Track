#include <iostream>
#include <fstream>
#include <math.h> 
#include <opencv2/highgui.hpp>
#include "Tracker.h"
#include "CameraEmulator.h"
#include "Utils/Timer.h"
#include "Utils/OutData.h"
#include "Utils/DetectorsData.h"

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::ofstream;

void printStats(const Stats& stats);
void toMatrix(cv::Mat in, double out[][3]);
void saveCsv(string filename, vector<OutData>& data);

int main(int argc, char* argv[]) {
    if(argc != 3) {
        cout << "ERROR: invalid number of arguments (required: 3, provided: " << argc << ")" << endl;
        return -1;
    }

    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

    // initialization
    CameraEmulator camera(argv[1]);

    cv::Ptr<cv::BFMatcher> m = cv::makePtr<cv::BFMatcher, int, bool>(cv::NORM_HAMMING, false);
    Tracker tracker(cv::BRISK::create(BRISK_THRESH, BRISK_OCTAVES, BRISK_PATTERN_SCALE), m);
    tracker.setWidth(camera.getWidth());
    tracker.setHeight(camera.getHeight());

    cout << "Frame time: " << camera.getFrameTime() << endl;
    cout << "Video time: " << camera.getVideoTime() << endl;

    // loop    
    Frame reference;
    Frame target;
    vector<OutData> output;
    Timer acquireTimer, computeTimer;
    double now = 0, latency;

    while(now < camera.getVideoTime()) {
        // acquiring last two frames
        acquireTimer.start();
        camera.acquireFrames(now, reference, target);
        cout << "Acquire time: " << acquireTimer.get() << " ms" << endl;

        // tracking the motion
        computeTimer.start();
        tracker.reset();
        tracker.setReference(reference);
        tracker.setTarget(target);
        tracker.track();
        latency = computeTimer.get();
        cout << "Compute time: " << latency << " ms" << endl;

        if(!tracker.good()) {
            now += latency;
            cout << endl;
            continue;
        }

        // output
        OutData data;
        data.time = now;
        data.motionAvg = tracker.getDistanceAvg();
        data.motionStd = tracker.getDistanceStd();
        data.angleAvg = tracker.getAngleAvg();
        data.angleStd = tracker.getAngleStd();
        toMatrix(tracker.getHomography(), data.homography);
        output.push_back(data);

        printStats(tracker.getStats());
        cout << endl;

        now += latency;
    }

    saveCsv(argv[2], output);

    return 0;
}

void printStats(const Stats& stats) {
    cout << "kp1: " << stats.kp1 << "\t\tkp2: " << stats.kp2 << endl;
    cout << "matches: " << stats.matches << "\t\tmatch ratio: " << stats.matches/(float)std::min(stats.kp1, stats.kp2) << endl;
    cout << "inliers: " << stats.inliers << "\t\tinlier ratio: " << stats.inliers/(float)stats.matches << endl;
}

void toMatrix(cv::Mat in, double out[][3]) {
    out[0][0] = in.at<double>(0, 0);
    out[0][1] = in.at<double>(0, 1);
    out[0][2] = in.at<double>(0, 2);
    out[1][0] = in.at<double>(1, 0);
    out[1][1] = in.at<double>(1, 1);
    out[1][2] = in.at<double>(1, 2);
    out[2][0] = in.at<double>(2, 0);
    out[2][1] = in.at<double>(2, 1);
    out[2][2] = in.at<double>(2, 2);
}

void saveCsv(string filename, vector<OutData>& data) {
    ofstream file;
    file.open(filename, std::ofstream::out);
    if(!file.is_open()) {
        cerr << "ERROR: failed to open the file " << filename << endl;
        return;
    }

    file << "Time,MotionAvg,MotionStd,DirectionAvg,DirectionStd,Homography" << endl;

    for(const OutData& o : data) {
        file << o.time << "," << o.motionAvg << "," << o.motionStd << "," << o.angleAvg << "," << o.angleStd;
        for(int i=0; i<3; ++i) {
            for(int j=0; j<3; ++j) {
                file << "," << o.homography[i][j];
            }
        }
        file << endl;
    }

    file.close();
}
