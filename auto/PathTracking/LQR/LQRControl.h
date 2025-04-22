#ifndef PMYproject_LQRCONTROL_H
#define PMYproject_LQRCONTROL_H

#define EPS 1.0e-4 
#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;


class LQRControl {
private:
    int N;

public:
    struct LQRResult {
        MatrixXd u;
        MatrixXd X;
    };

    LQRControl(int n);

    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    LQRResult lqrControl(vector<double>robot_state, vector<vector<double>>refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R, double v_ref);
};


#endif //PMYproject_LQRCONTROL_H
