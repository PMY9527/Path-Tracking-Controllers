#include "LQRControl.h"

LQRControl::LQRControl(int n) : N(n) {} // 使用初始化列表直接给成员变量N赋值，表明这个类需要指定迭代次数N来创建对象

/**
 * 解代数里卡提方程
 * @param A 状态矩阵A
 * @param B 状态矩阵B
 * @param Q Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
 * @param R R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。
 * @return
 */
MatrixXd LQRControl::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) {
    MatrixXd Qf = Q;
    MatrixXd P = Qf;
    MatrixXd P_; // P_new
    for(int i = 0; i < N; i++){
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        //小于预设精度时返回
        if((P_ - P).norm() < EPS){
            break;
            //cout << " LQR solved with #iteration: " << i << endl;
        }
        P = P_;
    }
    return P_;
}

/**
 * LQR控制器
 * @param robot_state
 * @param refer_path
 * @param s0
 * @param A
 * @param B
 * @param Q
 * @param R
 * @return
 */

LQRControl::LQRResult LQRControl::lqrControl(vector<double> robot_state, vector<vector<double>> refer_path, double s0, MatrixXd A, MatrixXd B,
                       MatrixXd Q, MatrixXd R, double v_ref) { //这是对外的主接口。states {x, y, psi, dot_psi, v}; 
    MatrixXd X(5,1);
    X<< robot_state[0]-refer_path[s0][0], // x 位置误差
        robot_state[1]-refer_path[s0][1], // y 位置误差
        robot_state[2]-refer_path[s0][2], // 横摆角误差
        robot_state[3]-refer_path[s0][3]*v_ref, // 横摆角速度误差
        robot_state[4]-v_ref;  // 速度误差
        
    MatrixXd P = calRicatti(A, B, Q, R);
    MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
    //MatrixXd u = K * X; //[v-ref_v,delta-ref_delta]
    
    LQRResult result;
    result.u = K * X;
    result.X = X; 
    return result;
}
