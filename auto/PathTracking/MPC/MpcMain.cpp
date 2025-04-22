#include "Mpc.h"
#include "../utils/MyReferencePath.h"
#include "../utils/KinematicModel.h"
#include "../utils/LowPassFilter.h"
#include "../utils/NormalizeAngle.hpp"
#include "../../matplotlibcpp.h"
#include <qpOASES.hpp>
namespace plt = matplotlibcpp;
using namespace Eigen;

int main(){

    USING_NAMESPACE_QPOASES

    MatrixXd A, Ad, Bd, Q_diag, R_diag, g, H;
    vector<double> robot_state;

    Q_diag.resize(1, 5);
    R_diag.resize(1, 2);

    // Q_diag: [x, y, ψ, ψ_dot, v]
    Q_diag << 12, 12, 6, 1, 1;

    // R_diag: [accel, steer_rate]
    R_diag << 1, 1; 

    double dt = 0.1; // 时间间隔，单位：s
    double L = 2; // 车辆轴距，单位：m
    double ref_v = 1; // 初始速度
    double x_0 = 0; // 初始x
    double y_0 = -1; // 初始y
    double psi_0 = 0; // 初始航向角
    double dot_psi_0 = 0; // 初始航向角速度
    double lpf_u0, lpf_u1, x_ref, y_ref;

    //保存运动过程中的轨迹
    vector<double> X0, X1, X2, X3, X4, u0, u1, t;
    vector<double> x_, y_, x_r, y_r;
    vector<double> psi_, v_;
    LowPassFilter lpf(0.67); // setting alpha 
    MyReferencePath referencePath;
    KinematicModel car(x_0, y_0, psi_0, dot_psi_0, ref_v, L, dt);
    Mpc Mpc;

    // 获取MPC问题维度
    int nVars = Mpc::nu * Mpc::mpc_N;  // 控制变量总数
    int nCons = 0;          // 约束数量

    // 创建QP问题实例
    QProblem qp(nVars, nCons);


    for(int i = 0; i < 1000; i++){
        plt::clf();
        robot_state = car.getState();
        MatrixXd X = Mpc.getX();
        psi_.push_back(robot_state[2]); // 航向角 psi
        v_.push_back(robot_state[4]);  // 速度 v
        vector<double> one_trial = referencePath.calcTrackError(robot_state); // 最近参考点的{error,k,yaw,min_index}
        double ref_yaw = one_trial[2]; // 最近参考点的yaw
        double s0 = one_trial[3]; // 最近参考点的索引

        vector<MatrixXd>state_space = car.stateSpace(ref_v, ref_yaw);
        Ad = state_space[0];
        Bd = state_space[1];
        Mpc.QPMatricesSetup(Ad,Bd,Q_diag,R_diag,robot_state,referencePath.refer_path,s0,ref_v); 
        H = Mpc.getHessian();
        g = Mpc.getGradient();
        
        // 将Eigen矩阵转换为qpOASES需要的数组格式
        real_t* H_qp = new real_t[nVars * nVars];
        real_t* g_qp = new real_t[nVars];
        MatrixXd H_lower = H.triangularView<Eigen::Lower>();
        Map<MatrixXd>(H_qp, nVars, nVars) = H_lower;
        Map<VectorXd>(g_qp, nVars) = g;

        // 设置控制输入约束
        real_t* lb = new real_t[nVars];
        real_t* ub = new real_t[nVars];
        for (int i = 0; i < nVars; ++i) {
            lb[i] = -10.0;
            ub[i] = 10.0; // TBD 这里都是 +-1，但为什么求解出来会有2.0？
        }
 
        int nWSR = 50; // 最大迭代次数
        qp.init(H_qp, g_qp, nullptr, nullptr, nullptr, lb, ub, nWSR);
        
        // 获取解
        real_t* u_opt = new real_t[nVars];
        qp.getPrimalSolution(u_opt);
        double accel = u_opt[0];
        double steer_rate = u_opt[1];
        delete[] u_opt;

        // 低通滤波器
        lpf_u0 = lpf.filter(accel);
        lpf_u1 = lpf.filter(steer_rate);

        // 清理内存
        delete[] H_qp;
        delete[] g_qp;
        delete[] lb;
        delete[] ub;
        qp.reset();

        car.updateState(lpf_u0, lpf_u1);
        t.push_back(i * dt);
        u0.push_back(lpf_u0);
        u1.push_back(lpf_u1);
        X0.push_back(X(0));
        X1.push_back(X(1));
        X2.push_back(X(2));
        X3.push_back(X(3));
        X4.push_back(X(4));
        //x_r.push_back(referencePath.refer_path[s0][0]);
        //y_r.push_back(referencePath.refer_path[s0][1]);
        x_ref = referencePath.refer_path[s0][0];
        y_ref = referencePath.refer_path[s0][1];
        x_.push_back(car.x);
        y_.push_back(car.y);
        plt::figure(1);
        plt::clf();
        plt::plot(referencePath.refer_x, referencePath.refer_y, "b--");
        plt::plot(x_, y_, "r");
        //plt::scatter(vector<double> {x_ref}, vector<double> {y_ref}, 50, {{"color", "red"}, {"marker", "*"}});

        plt::title("Path Tracking");
        plt::ylim(-5, 5);
        plt::grid(true);
        plt::pause(0.01);
        
        plt::figure(2);
        plt::clf();
        plt::named_plot("x Error (m)", t, X0);
        plt::named_plot("y Error (m)", t, X1);
        plt::named_plot("psi Error (rad)", t, X2);
        plt::legend();
        plt::title("Error Visualisation(x,y and psi)");
        plt::grid(true);
        plt::pause(0.001);

        plt::figure(3);
        plt::clf();
        plt::named_plot("dot_psi Error (rad/s)", t, X3);
        plt::named_plot("v Error (m/s)", t, X4);
        plt::legend();
        plt::title("Error Visualisation(psi_dot and v)");
        plt::grid(true);
        plt::pause(0.001);

        plt::figure(4);
        plt::clf();
        plt::named_plot("Acceleration (m/s^2)", t, u0);
        plt::named_plot("Yaw Angular Acceleration (rad/s^2)", t, u1);
        plt::legend();
        plt::title("Control Input Visualisation");
        plt::grid(true);
        plt::pause(0.001);
        
    }

    plt::figure(1);
    plt::save("./Tracking Visualisation(MPC).png");
    
    plt::figure(2);
    plt::save("./Error Visualisation1(MPC).png");
    plt::figure(3);
    plt::save("./Error Visualisation2(MPC).png");
    plt::figure(4);
    plt::save("./Control Input Visualisation(MPC).png");
    
    plt::show();
    return 0;
}