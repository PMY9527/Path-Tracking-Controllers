#ifndef PMYproject_KINEMATICMODEL_H
#define PMYproject_KINEMATICMODEL_H
#include <iostream>
#include <vector>
#include <cmath>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel {
public:
    double x, y, psi, dot_psi, v, L, dt;
public:
    KinematicModel();

    KinematicModel(double x, double y, double psi, double dot_psi, double v, double l, double dt);

   vector<double>getState();

    void updateState(double accel, double ddot_psi);

    vector<MatrixXd> stateSpace(double ref_v, double ref_yaw);

};


#endif //PMYproject_KINEMATICMODEL_H
