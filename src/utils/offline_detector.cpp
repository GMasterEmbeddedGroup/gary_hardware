#include "utils/offline_detector.hpp"

using namespace utils;

OfflineDetector::OfflineDetector(double duration) {
    this->offline = false;
    this->flag_update = false;
    this->_duration = duration;
    this->last_time = high_resolution_clock::now();
}

void OfflineDetector::config(double duration) {
    this->_duration = duration;
}

void OfflineDetector::update(bool state) {
    auto time_now = high_resolution_clock::now();
    if (duration_cast<duration<double>>(time_now - this->last_time).count() < this->_duration) {
        this->flag_update |= state;
        return;
    }
    this->offline = !this->flag_update;
    this->last_time = time_now;
    this->flag_update = false;
}
