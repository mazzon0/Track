#pragma once

#include <iostream>
#include <opencv2/core.hpp>

using std::cout;
using std::endl;

void concMatrices(const cv::Mat* in, size_t inSize, cv::Mat& out) {
    if(!in || inSize == 0) {
        out = cv::Mat(0, 0, CV_8UC1);
        return;
    }
    
    int totalRows=0, rows=0;
    for(size_t i = 0; i < inSize; ++i) {
        if(!(in[i].dims <= 2)) cout << "dimensions error" << endl;
        if(!(in[i].cols == in[0].cols)) cout << "columns error" << endl;
        if(!(in[i].type() == in[0].type())) cout << "types error" << endl;

        totalRows += in[i].rows;
    }

    out.create(totalRows, in[0].cols, in[0].type());

    for(size_t i = 0; i < inSize; ++i) {
        cv::Mat dpart(out, cv::Rect(0, rows, in[i].cols, in[i].rows));
        in[i].copyTo(dpart);
        rows += in[i].rows;
    }
}