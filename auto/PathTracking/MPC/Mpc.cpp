#include "Mpc.h"

Mpc::Mpc() noexcept {
    X_0.resize(5, 1); // 显式初始化维度
    X_0.setZero();    // 初始值为零
}

    void Mpc::QPMatricesSetup(MatrixXd Ad, MatrixXd Bd, MatrixXd Q_diag, MatrixXd R_diag, vector<double> robot_state, vector<vector<double>> refer_path, double s0, double v_ref){

        Q_diag_N.resize(1, nx * mpc_N);
        R_diag_N.resize(1, nu * mpc_N);
        Q_diag_N.setZero();
        R_diag_N.setZero();
        Q.resize(nx * mpc_N, nx * mpc_N);
        R.resize(nu * mpc_N, nu * mpc_N);
        Q.setZero();
        R.setZero();

        for (int i = 0; i < mpc_N; i++)
        {
            Q_diag_N.block<1, nx>(0, i * nx) = Q_diag;
        }

        for (int i = 0; i < mpc_N; i++)
        {
            R_diag_N.block<1, nu>(0, i * nu) = R_diag;
        }

        for (int i = 0; i < nx * mpc_N; i++)
        {
            Q(i, i) = Q_diag_N(0, i);
        }

        for (int i = 0; i < nu * mpc_N; i++)
        {
            R(i, i) = R_diag_N(0, i);
        }

        // 构造 MPC 预测时域内的 QP
        // Aqp = [  A,
        //         A^2,
        //         A^3,
        //         ...
        //         A^k]' 
        // with a size of (nx * mpc_N, nx)

        // Bqp = [A^0*B,        0,       0,   ...       0 
        //         A^1*B,       B,            ...
        //         A^2*B,     A*B,       B,   ...       0
        //         ...
        //         A^(k-1)*B, A^(k-2)*B, A^(k-3)*B, ... B]  
        // with a size of (nx * mpc_N, nu * mpc_N)

        //X_0.resize(5,1);
        X_0 << robot_state[0]-refer_path[s0][0], // x 位置误差
        robot_state[1]-refer_path[s0][1], // y 位置误差
        robot_state[2]-refer_path[s0][2], // 横摆角误差
        robot_state[3]-refer_path[s0][3]*v_ref, // 横摆角速度误差
        robot_state[4]-v_ref;  // 速度误差

        Bqp.resize(nx * mpc_N, nu * mpc_N);
        Bqp.setZero();
        for (int i = 0; i < mpc_N; ++i) {
        // 计算 A^i
        MatrixXd Ad_power = MatrixXd::Identity(nx, nx);
        for (int k = 0; k < i; ++k) {
                Ad_power *= Ad;
            }
            Aqp.block(nx * i, 0, nx, nx) = Ad_power;

            // 填充 Bqp
            for (int j = 0; j <= i; ++j) {
                MatrixXd Ad_power_j = MatrixXd::Identity(nx, nx);
                for (int k = 0; k < i - j; ++k) {
                    Ad_power_j *= Ad;
                }
                Bqp.block(nx * i, nu * j, nx, nu) = Ad_power_j * Bd;
            }
        }
        dense_hessian.resize(nu * mpc_N, nu * mpc_N);
        dense_hessian.setZero();
        dense_hessian = (Bqp.transpose() * Q * Bqp); 
        dense_hessian += R;
        dense_hessian = dense_hessian * 2;
        
        gradient.setZero();
        gradient = 2 * Bqp.transpose() * Q * Aqp * X_0;

    }
    const MatrixXd& Mpc::getHessian() const {
        return dense_hessian;
    }
    const MatrixXd& Mpc::getGradient() const {
        return gradient;
    }
    const MatrixXd& Mpc::getX() const {
        return X_0;
    }
