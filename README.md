## 1. System Modeling and Linearization

### 1.1 Extended Kinematic Model
We extend the bicycle model to a 5th-order system:

$$ 
\mathbf{X} = \begin{bmatrix} x \\ y \\ \psi \\ \dot{\psi} \\ v \end{bmatrix}, \quad
\mathbf{U} = \begin{bmatrix} a \\ \ddot{\psi} \end{bmatrix}
$$

State equations:
$$
\begin{cases}
\dot{x} = v\cos\psi \\
\dot{y} = v\sin\psi \\
\dot{\psi} = \dot{\psi} \\
\ddot{\psi} = \ddot{\psi} \\
\dot{v} = a
\end{cases}
$$

### 1.2 Linearization
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

## 2. Controller Design

### 2.1 LQR Implementation
Cost function:
$$
\min_{\mathbf{u}} J = \sum_{k=1}^N (\mathbf{\tilde{X}}_k^T Q \mathbf{\tilde{X}}_k + \mathbf{\tilde{U}}_k^T R \mathbf{\tilde{U}}_k)
$$

Optimal control law:
$$
\mathbf{u} = -K\mathbf{\tilde{X}}, \quad K = (R + B_d^T P B_d)^{-1} B_d^T P A_d
$$

### 2.2 MPC Formulation
Receding horizon optimization:
$$
\begin{aligned}
\min_{\mathbf{U}} & \sum_{k=0}^{N_p} \mathbf{\tilde{X}}_{k|t}^T Q \mathbf{\tilde{X}}_{k|t} + \sum_{k=0}^{N_c-1} \mathbf{\tilde{U}}_{k|t}^T R \mathbf{\tilde{U}}_{k|t} \\
\text{s.t.} & \quad \mathbf{\tilde{X}}_{k+1|t} = A_d \mathbf{\tilde{X}}_{k|t} + B_d \mathbf{\tilde{U}}_{k|t}
\end{aligned}
