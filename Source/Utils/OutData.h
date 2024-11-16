#pragma once

#include <vector>

using std::vector;

struct OutData {
    double time, motionAvg, motionStd, angleAvg, angleStd;
    double homography[3][3];
};

double average(const vector<double>& v);
double stdDeviation(const vector<double>& v, const double average);
void computeStats(const vector<double>& v, double& outAvg, double& outStd);
