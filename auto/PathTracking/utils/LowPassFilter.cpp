#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double alpha, double init_val)
    : alpha_(alpha), y_prev_(init_val) {}

double LowPassFilter::filter(double x) {
    double y = alpha_ * x + (1.0 - alpha_) * y_prev_;
    y_prev_ = y;
    return y;
    // low pass filter design
    // alpha = 2 * pi * cutoff_freq * dt / (1 + 2 * pi * cutoff_freq * dt)
    // cutoff_freq = 1 / 2 * pi * tau, with tau = 50 ms, dt = 0.01 s, alpha = 0.17 ----- TBD：看看tau怎么来的
    // tau = 50 ms, dt = 0.1 s, alpha = 0.67 
    // tau = 20 ms, dt = 0.01 s, alpha = 0.33
}
