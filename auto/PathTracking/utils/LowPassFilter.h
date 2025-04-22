#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter {
public:
    LowPassFilter(double alpha, double init_val = 0.0);
    double filter(double x);

private:
    double alpha_;
    double y_prev_;
};

#endif
