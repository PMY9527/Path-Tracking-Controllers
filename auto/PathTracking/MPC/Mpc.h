#ifndef MPC_H
#define MPC_H

#include <vector>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;


class Mpc {
    public:

    static const int mpc_N = 10; // MPC 预测区间
    static const int nx = 5;    // 状态向量的维数
    static const int nu = 2;    // 控制输入的维数

    Mpc() noexcept; 

    void QPMatricesSetup(MatrixXd Ad, MatrixXd Bd, MatrixXd Q_diag, MatrixXd R_diag, vector<double> robot_state, vector<vector<double>> refer_path, double s0, double v_ref);
    const MatrixXd& getHessian() const;
    const MatrixXd& getGradient() const;
    const MatrixXd& getX() const;

    private:
    Matrix<double, nx * mpc_N, nu> Bd_list;
    Matrix<double, nx * mpc_N, nx> Aqp;
    MatrixXd X_0,gradient, Bqp, dense_hessian, Q_diag_N, R_diag_N, Q, R;

    //Matrix<double, nx, 1> X_0;
};

#endif // MPC_H