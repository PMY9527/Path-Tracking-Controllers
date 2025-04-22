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


