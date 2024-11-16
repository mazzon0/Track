#include "OutData.h"

#include <iostream>
#include <math.h>
using std::cout;
using std::endl;

double average(const vector<double>& v) {
    double sum = 0;
    size_t nans = 0;

    for(double d : v) {
        if(!isnan(d)) {
            sum += d;
        }
        else {
            ++nans;
        }
    }

    return sum / (v.size()-nans);
}

double stdDeviation(const vector<double>& v, const double average) {
    double sum = 0;
    size_t nans = 0;
    for(double d : v) {
        if(!isnan(d)) {
            sum += d*d;
        }
        else {
            ++nans;
        }
    }
    return std::sqrt((sum/v.size() - average*average) / (v.size()-nans-1));
}

void computeStats(const vector<double>& v, double& outAvg, double& outStd) {
    outAvg = average(v);
    outStd = stdDeviation(v, outAvg);
}
