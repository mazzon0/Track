#pragma once

#include <chrono>

class Timer {
public:
    void start() {s = std::chrono::high_resolution_clock::now();}
    double get() {
        std::chrono::high_resolution_clock::time_point e = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(e-s).count();
    }
private:
    std::chrono::high_resolution_clock::time_point s;
};