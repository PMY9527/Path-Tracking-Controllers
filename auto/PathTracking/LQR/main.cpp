#include "LQRControl.h"
#include "../utils/MyReferencePath.h"
#include "../utils/KinematicModel.h"
#include "../utils/LowPassFilter.h"
#include "../utils/NormalizeAngle.hpp"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
    vector<double> X0, X1, X2, X3, X4, u0, u1, t;
    double lpf_u0, lpf_u1;
    double dt = 0.1; // 时间间隔，单位：s
    double L = 2; // 车辆轴距，单位：m
    double v = 1; // 初始速度
    double x_0 = 0; // 初始x
    double y_0 = -1; // 初始y
    double psi_0 = 0; // 初始航向角
    double dot_psi_0 = 0; // 初始航向角速度
    int N = 500; // lqr求解迭代范围

    MatrixXd Q(5,5);
        Q << 12, 0, 0, 0, 0,
             0, 12, 0, 0, 0,
             0, 0, 3, 0, 0,
             0, 0, 0, 3, 0,
             0, 0, 0, 0, 3;
    MatrixXd R(2,2);
        R << 0.1, 0,
             0, 0.1;

    //保存运动过程中的轨迹
    vector<double> x_, y_;
    vector<double> psi_, v_;
    //vector<double> x_r, y_r;
    LowPassFilter lpf(0.67); // setting alpha 
    MyReferencePath referencePath;
    KinematicModel car(x_0, y_0, psi_0, dot_psi_0, v, L, dt);
    LQRControl lqr(N);
    vector<double> robot_state;

    for(int i = 0; i < 1000; i++){
        plt::clf();
        robot_state = car.getState(); // return {x, y, psi, dot_psi, v};
        psi_.push_back(robot_state[2]); // 航向角 psi
        v_.push_back(robot_state[4]);  // 速度 v
        vector<double> one_trial = referencePath.calcTrackError(robot_state); // 最近参考点的{error,k,yaw,min_index}
        double ref_yaw = one_trial[2]; // 最近参考点的yaw
        double s0 = one_trial[3]; // 最近参考点的索引，用于下方lqr.lqrControl
        double ref_v = v;
        //double ref_delta = atan2(L*k,1);
        vector<MatrixXd>state_space = car.stateSpace(ref_v, ref_yaw); // state_space[0] 为 A 矩阵 state_space[1] 为 B 矩阵 
        LQRControl::LQRResult res = lqr.lqrControl(robot_state, referencePath.refer_path, s0, state_space[0], state_space[1], Q, R, ref_v);
        MatrixXd U = res.u;
        MatrixXd X = res.X;
        lpf_u0 = lpf.filter(U(0));
        lpf_u1 = lpf.filter(U(1));
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
        x_.push_back(car.x);
        y_.push_back(car.y);
        plt::figure(1);
        plt::clf();
        plt::plot(referencePath.refer_x, referencePath.refer_y, "b--");
        plt::plot(x_, y_, "r");
        plt::title("Path Tracking");
        plt::ylim(-5, 5);
        plt::grid(true);
        plt::pause(0.001);

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
    // save figure

    plt::figure(1);
    plt::save("./Tracking Visualisation(LQR).png");
    plt::figure(2);
    plt::save("./Error Visualisation1(LQR).png");
    plt::figure(3);
    plt::save("./Error Visualisation2(LQR).png");
    plt::figure(4);
    plt::save("./Control Input Visualisation(LQR).png");
    plt::show();
    return 0;
}