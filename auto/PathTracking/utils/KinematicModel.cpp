#include "KinematicModel.h"
/**
 * 机器人运动学模型构造
 * @param x 位置x
 * @param y 位置y
 * @param psi 偏航角
 * @param dot_psi 偏航角速度
 * @param v 速度
 * @param l 轴距
 * @param dt 采样时间
 */
KinematicModel::KinematicModel(double x, double y, double psi, double dot_psi, double v, double l, double dt) : x(x), y(y), psi(psi), dot_psi(dot_psi),
                                                                                                v(v), L(l), dt(dt) {}
/**
 * 控制量为偏航角加速度 ddot_psi 和 加速度 a
 * @param accel 加速度
 * @param ddot_psi 偏航角加速度
 */
void KinematicModel::updateState(double accel, double ddot_psi) { // STATES: [x, y, psi, psi_dot, v]
    x = x + v*cos(psi)*dt;
    y = y + v*sin(psi)*dt;
    dot_psi = dot_psi + ddot_psi*dt;
    psi = psi + dot_psi*dt;
    //psi = psi + v / L * tan(delta_f)*dt;
    v = v + accel*dt;
}

/**
 * 状态获取
 * @return
 */
vector<double> KinematicModel::getState() {
    return {x, y, psi, dot_psi, v};
}

/**
 * 将模型离散化后的状态空间表达
 * @param ref_v 参考速度
 * @param ref_yaw 参考偏航角
 * @return
 */
vector<MatrixXd> KinematicModel::stateSpace(double ref_v, double ref_yaw) {
    MatrixXd A(5,5), B(5,2);
    A << 0, 0, -ref_v*sin(ref_yaw), 0, cos(ref_yaw),
         0, 0,  ref_v*cos(ref_yaw), 0, sin(ref_yaw),
         0, 0,  0,                  1, 0,
         0, 0,  0,                  0, 0,
         0, 0,  0,                  0, 0;

    B << 0, 0,
         0, 0,
         0, 0,
         0, 1,
         1, 0;
    A = MatrixXd::Identity(5,5) + A * dt;
    B = dt * B;
    
    return {A, B};
}


