## System Modeling

### Extended Kinematic Model
The bicycle model is extended to a 5th-order system:

$$ 
\mathbf{X} = \begin{bmatrix} x, \\ y, \\ \psi, \\ \dot{\psi}, \\ v, \end{bmatrix} \quad
\mathbf{U} = \begin{bmatrix} a, \\ \ddot{\psi} \end{bmatrix}
$$

$$
\begin{cases}
\dot{x} = v\cos\psi \\
\dot{y} = v\sin\psi \\
\dot{\psi} = \dot{\psi} \\
\ddot{\psi} = \ddot{\psi} \\
\dot{v} = a
\end{cases}
$$

### Linearization & Discretization
Jacobian matrices at reference point $(X_{ref}, U_{ref})$:

$$
A = \begin{bmatrix}
0 & 0 & -v_{ref}\sin\psi_{ref} & 0 & \cos\psi_{ref} \\
0 & 0 & v_{ref}\cos\psi_{ref} & 0 & \sin\psi_{ref} \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 1 \\
1 & 0
\end{bmatrix}
$$

Discrete-time formulation (Forward Euler):

$$
A_d = I + \Delta t A, \quad B_d = \Delta t B
$$

## Results
### LQR

<div style="display: inline-block; width: 15%; margin-right: 5%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/LQR_PICS/直线/Tracking%20Visualisation(LQR).png" alt="LQR 直线跟踪" width="100%" />
</div>
<div style="display: inline-block; width: 15%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/LQR_PICS/直线/Control%20Input%20Visualisation(LQR).png" alt="LQR 直线控制输入" width="100%" />
</div>

<div style="display: inline-block; width: 15%; margin-right: 5%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/LQR_PICS/曲线/Tracking%20Visualisation(LQR).png" alt="LQR 曲线跟踪" width="100%" />
</div>
<div style="display: inline-block; width: 15%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/LQR_PICS/曲线/Control%20Input%20Visualisation(LQR).png" alt="LQR 曲线控制输入" width="100%" />
</div>

### MPC

<div style="display: inline-block; width: 15%; margin-right: 5%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/MPC_PICS/直线/StraightLine_MPC.png" alt="MPC 直线跟踪" width="100%" />
</div>
<div style="display: inline-block; width: 15%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/MPC_PICS/直线/Control%20Input%20Visualisation(MPC).png" alt="MPC 直线控制输入" width="100%" />
</div>

<div style="display: inline-block; width: 15%; margin-right: 5%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/MPC_PICS/曲线/Tracking%20Visualisation(MPC).png" alt="MPC 曲线跟踪" width="100%" />
</div>
<div style="display: inline-block; width: 15%;">
  <img src="https://github.com/PMY9527/Path-Tracking-Controllers/blob/main/MPC_PICS/曲线/Control%20Input%20Visualisation(MPC).png" alt="MPC 曲线控制输入" width="100%" />
</div>



