# Tyr
A Cart-Pole Stabilization Project

![Tyr](videos\cart-pole.gif)

The Cart-Pole Stabilization problem is a benchmark problem for control algorithms. The problem involves keeping a pendulum upright in the presence of disturbances and inherent instability.
This project is a simulation utilizing ROS2 and Gazebo Sim. This is intended to serve as an in-depth technical paper explaining the project, design choices and methodology.


This project can be split into 3 parts: the stl files creation, the matlab system design and the ros implementation. The CAD files were designed for quick simulation and can be 
found in the src\pendulum\description\meshes directory. The MATLAB system design and ROS implementation are discussed below.

<a name=matlab><\a>
## MATLAB
### Mathematical Modelling 
![Cart-pole](images\cartpole.jpg)

**Variables**
$$
l = pendulum length
m_c = cart mass
m_p = pendulum mass
x_c = cart position
x_p = pendulum horizontal position
y_p = pendulun vertical position
\dot{x_c} = cart velocity
\dot{x_p} = pole horizontal velocity
\dot{y_p} = pole vertical velocity
\theta = pole angular displacement from vertical axis
g = acceleration due to gravity, 9.81ms^{-2}
T = Rod tension 
$$

$$
First, we get the horizontal and vertical accelerations for the pendulum center of mass (which is at its tip)
x_p = x_c + lsin\theta
\dot{x_p} = \dot{x_c} + l\dot{\theta}cos\theta

\begin{equation}
\ddot{x_p} = \ddot{x_c} + l\ddot{\theta}cos\theta - l\dot{theta}^2sin\theta
\label{horizontal_accel}
\end{equation}

y_p =lcos\theta
\dot{y_p} = -l\dot{\theta}sin\theta
\begin{equation}
\ddot{y_p} = -l\ddot{\theta}sin\theta - l\dot{\theta}^2cos\theta
\label{vertical_acceleration}
\end{equation}

Applying Newton's laws:
\begin{equation}
\sum_{}^{} F_{xp} = m_p\ddot{x}_p = Tsin\theta
\label{horiz_force_sum}
\end{equation}
\begin{equation}
\sum_{}^{} F_{yp} = m_p\ddot{y}_p = Tcos\theta - m_pg
\label{vertical_force_sum}
\end{equation}
\begin{equation}
\sum_{}^{} F_{xc} = m_c\ddot{x}_c = F_x - Tsin\theta
\label{cart_force_sum}
\end{equation}

Substituting equ(\ref{horizontal_accel}) into equ(\ref{horiz_force_sum}):
\begin{equation}
m_p(\ddot{x}_c + l\ddot{\theta}cos\theta - l\theta^2sin\theta) = Tsin\theta
\label{Tsin}
\end{equation}

\begin{equation}
m_p(-l\ddot{\theta}sin\theta - l\dot{\theta}^2cos\theta) = Tcos\theta - mg
\label{Tcos}
\end{equation}

Dividing equ(ref\{Tcos}) by equ(ref\{Tsin}) and cross-multiplying yields:
\begin{equation}
cos\theta(\dot{x}_c+l\ddot{\theta}cos\theta - l\dot{\theta}^2sin\theta) = sin\theta(-l\ddot{\theta}sin\theta - l\dot{\theta}^2cos\theta) + mgsin\theta
\label{cross_mult_result}
\end{equation}

Simplifying:
\ddot{x}_ccos\theta + l\ddot{\theta}cos^2\theta= -l\ddot{\theta}sin^2\theta + gsin{\theta}
\ddot{x}_ccos\theta = -l\ddot{\theta}(cos^2\theta + sin^2\theta) + gsin\theta

From trig identities, (cos^2\theta + sin^2\theta) = 1

\begin{equation}
\therefore gsin\theta - l\ddot{\theta} - \ddot{x}_ccos\{\theta} = 0
\label{sig_equ_1}
\end{equation}

Substituting equ(\ref{Tsin}) into equ(\ref{cart_force_sum}):
m_c\ddot{x}_c = F_x - \ddot{x}_cm_p - m_pl\ddot{\theta}cos{\theta} + m_pl\dot{\theta}^2sin\theta
F_x = \ddot{x}_c(m_p+m_c) + \ddot{\theta}(m_plcos\theta) - m_pl\dot{\theta}^2sin\theta







