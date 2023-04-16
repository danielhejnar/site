---
title:Minimum-Fuel Planar Earth-to-Mars Low-Thrust Trajectories Using Bang-Bang Control
date: "2015-05-28T22:40:32.169Z"
description: Low-thrust Trajectory Optimization
---

# Minimum-Fuel Planar Earth-to-Mars Low-Thrust Trajectories Using Bang-Bang Control
# Introduction

Electric propulsion systems have demonstrated that low-thrust engines have the capability to be used for long-duration travels by the planetary and interplanetary space missions. Electric propulsion has been used by NASA's Deep space 1 [1] and Dawn [2], ESA's SMART-1 [3], and JAXA's Hayabusa and Hayabusa 2 [4]. Electric propulsion systems have demonstrated that lowthrust engines can be used for long-duration travels by the planetary and interplanetary space missions [5]. Low-thrust electric propulsion spacecraft is known to have a greater payload capability than conventional chemical propulsion spacecraft. Low-thrust trajectory optimization generally accompanies determining the control variables, which may include thrust magnitude, and direction and parameter, and the corresponding trajectories while minimizing a given performance index (propellant consumption or time-of-flight) and satisfying boundary conditions (departure and arrival orbits), mid-point conditions, and path constraints.

Low-thrust trajectory optimization problems can be generally solved by either indirect or direct method [6]. The optimization of low-thrust trajectories have been mathematically formulated as an Optimal Control Problem (OCP) [7]. Indirect methods solve the optimal control problem by obtaining the solution to the corresponding two-point boundary value problem (TPBVP) which results from the calculus of variations. The application of the Lagrange multipliers (costates) doubles the number of differential equations that have to be integrated along the trajectory. However, the solution to the TPBVP is very sensitive to initial guess for costate variables which do have any intuitive physical meanings. To overcome the disadvantage in the indirect methods, heuristic and/or evolutionary techniques have been developed [8,9]. In contrast, direct methods solve the optimal control problem by converting it into a nonlinear programming (NLP) problem with various transaction schemes applied to either states and controls, or both states and controls. Since the control variables are explicitly parameterized, a good initial control guess for direct methods can be easily produced. In direct methods, the modifications of the performance index, equality constraints, state and control inequality constraints can easily made for different problem formulations while they should result in a new derivation of TPBVP in indirect methods.

This paper presents indirect and direct methods for obtaining fuel-optimal planar Earth-to-Mars trajectories using CSI engines where the differential equations of motion are numerically integrated and the continuous- time control is parameterized. Thus, the purpose of this paper is to develop an indirect and direct optimization methods to solve minimum-fuel planar low-thrust Earth-to-Mars trajectories in the heliocentric frame with bang-bang control structure. In addition, the solutions of this problem using the indirect and direct methods will be validated by comparing them at the different flight time and maximum powers of the spacecraft and analyzing the optimization results. A performance index representing propellant consumed for CSI engines is formulated to minimize its total propellant consumption, resulting in on-off-on thrusting sequence by Lawden's primer vector theory $[10,11]$. Minimizing the performance index is equivalent to maximizing the final mass of the spacecraft. The formulation of the problem treats the spacecraft mass as a state variable, thus updating the spacecraft mass to the optimal trajectory design. For both optimization methods, the equations of the motion are normalized for the fundamental distance, velocity, mass and time to streamline the numerical computations. In the the indirect method, the analytical formulations of the problem are presented to set up TPBVP using the primer vector theory, and the necessary conditions for an optimal solution are discussed. In the direct method, the bounds of the states, the flight time, and the control including the maximum available power, the equality constraint and the boundary conditions are explicitly specified. The optimal control problem is then converted to the parameter optimization problem that can be solved by nonlinear programming (NLP). As the NLP solver, a general-purpose optimal control software called GPOPS-II [12] is adopted to solve optimal control problem using variable-order Gaussian quadrature collocation methods where the continuous-time optimal control problem is transcribed to a large sparse nonlinear programming problem.

Numerical studies show that the fuel-optimal, low-thrust heliocentric Earth-to-Mars trajectories for the specific arrival time and maximum power are obtained with different thrust magnitudes to find "on-off-on" thrusting sequence. The primer vector theory is employed to analyze the control structure by monitoring the variations of the switching functions. Finally, the fuel-optimal planar Earth-to-Mars trajectories at different flight time and maximum power are validated.

# Problem Formulation

In this section, the governing equations of motion are given and the scaled equations of motion are derived. The spacecraft is propelled by electric propulsion.

## Equations of Motion

This trajectory optimization problem is modeled in a two-dimensional, heliocentric (suncentered) polar coordinate system to make the NLP problem both efficient and robust instead of Cartesian coordinates which is the simplest but most disadvantageous choice [13]. All motions are assumed to be confined to the ecliptic or fundamental plane. Figure 1 illustrates the geometry of this coordinate system along with the steering angle.

where $r$ is the heliocentric radius of the spacecraft, $\theta$ is the phase angle, $u$ and $v$ are the radial and transverse velocity components, respectively, and $\alpha$ is the steering or thrust orientation angle. The steering angle is measured relative to the instantaneous tangential direction and is positive in the

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-03.jpg?height=602&width=902&top_left_y=307&top_left_x=814)

Figure 1. Geometry of heliocentric coordinate system along with the steering angle.

clockwise direction to the thrust vector. The two-dimensional equations of motion for the spacecraft in the polar coordinate system $[13,14]$ are given by

$$
\begin{aligned}
\dot{r} & =v_{r}, \\
\dot{v}_{r} & =\frac{v_{\theta}^{2}}{r}-\frac{\mu}{r^{2}}+\frac{T}{m} u_{1}, \\
\dot{v}_{\theta} & =-\frac{v_{r} v_{\theta}}{r}+\frac{T}{m} u_{2}, \\
\dot{\theta} & =\frac{v_{\theta}}{r}, \\
\dot{m} & =-\frac{T}{c},
\end{aligned}
$$

where $m$ is time varying spacecraft mass, $u_{1}$ and $u_{2}$ are the unit vector components in the thrust vector, $\mu$ is the solar gravitational constant, which equals to $1.32712441933 \times 10^{11}, T$ represents the thrust magnitude and $c=I_{s p} g_{0}$ is the exhaust velocity. $I_{s p}$ and $g_{0}=9.80665 \mathrm{~m} / \mathrm{s}^{2}$ are the specific impulse and the standard gravitational acceleration at sea level, respectively. The radial and tangential acceleration components due to thrust are defined ad

$$
\begin{aligned}
a_{r} & =\frac{T}{m} \sin \alpha \\
a_{\theta} & =\frac{T}{m} \cos \alpha
\end{aligned}
$$

The boundary conditions and the inequality should be specified for a complete optimization problem. Here, the initial boundary conditions can be formulated as

$$

\begin{aligned}
    r(0)&=r_{0}, \\
 v_{r}(0)&=v_{r_{0}}=0, \\
 v_{\theta}(0)&=v_{\theta_{0}}=\sqrt{\frac{\mu}{r_{0}}},\\
 \theta(0)&=\theta_{0}=0, \\
 m(0)&=m_{0}
\end{aligned}
$$

while the terminal boundary conditions are given by

$$
\begin{aligned}
    r\left(t_{f}\right)&=r_{f}, \\
    v_{r}\left(t_{f}\right)&=v_{r_{f}}=0, \\
    v_{\theta}\left(t_{f}\right)&=\sqrt{\frac{\mu}{r_{f}}}
\end{aligned}
$$

The first equation in Eq. (5) states that the radial velocity should be zero and the second equation is a boundary condition is that forces the final velocity to be equal to the local circular velocity at the final radial distance. The equality path constraint is given by

$$
e=u_{1}^{2}+u_{2}^{2}=1
$$

## Scaled Equations of Motion

We scale the state variables to remain close to unity to streamline numerical computations. All heliocentric distances are normalized with respect to the $\mathrm{AU}$ (Astronomical Unit) which is equal to $149597870.691 \mathrm{~km}$. Likewise, all velocity values are normalized with respect to the "local circular velocity", $v_{\theta_{0}}=\sqrt{\mu / r_{0}}$ at the heliocentric distance of the Earth's circular orbit, $r_{0}$. Using the initial orbit radius and initial velocity as reference values, and the normalizing the time by the final time, we define the new state variables as:

$$
\begin{aligned}
\bar{r}&=\frac{r}{r_{0}},\\
 \bar{v}_{r}&=\frac{v_{r}}{v_{\theta_{0}}}, \\
 \bar{v}_{\theta}&=\frac{v_{\theta}}{v_{\theta_{0}}}, \\
 \tau&=\frac{t}{t_{f}} .
\end{aligned}
$$

Applying the chain rule to change variables results in the following new non-dimensional equations of motion:

For $\bar{r}$ :

$$
\frac{d \bar{r}}{d \tau}=\frac{d r / r_{0}}{d t / t_{f}}=\frac{d r}{d t} \frac{t_{f}}{r_{0}}=v_{r} \frac{t_{f}}{r_{0}}=\frac{v_{r}}{v_{\theta_{0}}} v_{\theta_{0}} \frac{t_{f}}{r_{0}}=\bar{v}_{r} \zeta
$$

where $\zeta=\frac{v_{\theta_{0}} t_{f}}{r_{0}}$

For $\bar{v}_{r}$

$$
\begin{aligned}
\frac{d \bar{v}_{r}}{d \tau} & =\frac{d\left(v_{r} / v_{\theta_{0}}\right)}{d t} \frac{d t}{d \tau}=\dot{v}_{r} \frac{t_{f}}{v_{\theta_{0}}}=\left(\frac{v_{\theta}^{2}}{r}-\frac{\mu}{r^{2}}+\frac{T u_{1}}{m_{0}+\dot{m} t}\right) \frac{t_{f}}{v_{\theta_{0}}} \\
& =\left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}}-\frac{1}{\bar{r}^{2}}\right) \zeta+\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right) u_{1},
\end{aligned}
$$

where $a=\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right)$.

For $\bar{v}_{\theta}:$

$$
\begin{gathered}
\frac{d \bar{v}_{\theta}}{d \tau}=\frac{d v_{\theta} / v_{\theta_{0}}}{d t / t_{f}}=\frac{d v_{\theta}}{d t} \frac{t_{f}}{v_{\theta_{0}}}=\dot{v}_{\theta} \frac{t_{f}}{v_{\theta_{0}}}=\left(-\frac{v_{\theta} v_{\theta}}{r}+\frac{T u_{2}}{m}\right) \frac{t_{f}}{v_{\theta_{0}}}, \\
\frac{d \bar{v}_{\theta}}{d \tau}=-\frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}} \zeta+\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right) u_{2} .
\end{gathered}
$$

For $\bar{\theta}$ :

$$
\frac{d \theta}{d \tau}=\frac{d \theta}{d t} \frac{d t}{d \tau}=\frac{d \theta}{d t} t_{f}=\frac{v_{\theta}}{r} t_{f}=\frac{\bar{v}_{\theta}}{\bar{r}} \zeta .
$$

For $\tilde{m}$ :

$$
\frac{d \tilde{m}}{d \tau}=\frac{d m}{d t} \frac{t_{f}}{m_{0}}=\left(\frac{-T}{c}\right)\left(\frac{t_{f}}{m_{0}}\right)=\left(\frac{-2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right) .
$$

In summary, newly scaled equations of the motion are:

$$
\begin{aligned}
&\frac{d \bar{r}}{d \tau} = \bar{v}_{r} \zeta \\
&\frac{d \bar{v}_{r}}{d \tau} = \left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}}-\frac{1}{\bar{r}^{2}}\right) \zeta+\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right) u_{1} \\
&\frac{d \bar{v}_{\theta}}{d \tau} = -\frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}} \zeta+\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right) u_{2} \\
&\frac{d \theta}{d \tau} = \frac{\bar{v}_{\theta}}{\bar{r}} \zeta \\
& \frac{d \tilde{m}}{d \tau} = \left(\frac{-2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right) .
\end{aligned}
$$

The scaled initial boundary conditions can be formulated as

$$
\begin{aligned}

&r(0)=1, \\
&v_{r}(0)=v_{r_{0}}=0, \\
&v_{\theta}(0)=v_{\theta_{0}}=1, \\
&\theta_{0}=0, \\
&\tilde{m}(0)=1
\end{aligned}
$$

while the scaled terminal boundary conditions are given by

$$
\begin{aligned}
    &r\left(t_{f}\right)=\frac{r_{f}}{r_{0}}, \\
    &v_{r}\left(t_{f}\right)=v_{r_{f}}=0, \\
    &v_{\theta}\left(t_{f}\right)=\sqrt{\frac{r_{0}}{r_{f}}}

\end{aligned}
$$

# Low-Thrust Trajectory Optimization

The trajectory is controlled by the thrust magnitude and direction. The thrust level

$$
T=\frac{2 \eta P}{c}
$$

is obtained by selecting exhaust velocity and the engine input power $P$ (thruster efficiency $\eta$ is assumed constant throughout), taking the operational constraints int account. The input power is always limited by the availability of the solar power $\left(P \leq P_{\max }\right)$ and the exhaust velocity is constant. The payload is the performance index to be maximized. For the minimum-fuel problem with CSI engines, the performance index can be established as

$$
J=-\tilde{m}_{f}
$$

It is obvious from Eq. (15) that minimizing $J$ is equivalent to maximizing the final mass $m_{f}$ or the payload.

## Indirect method

The Hamiltonian is formulated based on the Pontryain's minimum principle (PMP):

$$
H=\lambda_{\bar{r}} \bar{v}_{r}+\lambda_{\bar{v}_{r}}\left[\left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}}-\frac{1}{\bar{r}^{2}}\right)+a u_{1}\right]+\lambda_{\bar{v}_{\theta}}\left(-\frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}}+a u_{2}\right)+\lambda_{\theta} \frac{\bar{v}_{\theta}}{\bar{r}}+\lambda_{\tilde{m}}\left(\frac{-2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right),
$$

where $\lambda=\left[\lambda_{\bar{r}}, \lambda_{\bar{v}_{r}}, \lambda_{\bar{v}_{\theta}}, \lambda_{\theta}\right]^{T}$ are the costate vectors adjoint to the radius, radial velocity, transverse velocity, $\theta$, and mass, respectively.

$$
\begin{aligned}
H= & \lambda_{\bar{r}} \bar{v}_{r}+\lambda_{\bar{v}_{r}}\left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}}-\frac{1}{\bar{r}^{2}}\right)-\lambda_{\bar{v}_{\theta}}\left(\frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}}\right)+\lambda_{\theta} \frac{\bar{v}_{\theta}}{\bar{r}} \\
& +\left[a \lambda_{\bar{v}_{r} \bar{v}_{\theta}} u+\lambda_{\tilde{m}}\left(\frac{-2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right)\right],
\end{aligned}
$$

where $\lambda_{\bar{v}_{r} \bar{v}_{\theta}}=\left[\lambda_{\bar{v}_{r}}, \lambda_{\bar{v}_{\theta}}\right]^{T}$ and $\boldsymbol{u}=\left[u_{1}, u_{2}\right]^{T}$. The optimal control law is derived to minimize the Hamiltonian. Thus, the direction of the thrust is opposite to the adjoint vector $\lambda_{\bar{v}_{r} \bar{v}_{\theta}}$ and the primer vector $\boldsymbol{p}(t)$ is defined, so that

$$
\lambda \bar{v}_{r} \bar{v}_{\theta} \boldsymbol{u}=-\lambda_{\bar{v}_{r} \bar{v}_{\theta}}=-p(t)
$$

for which

$$
\boldsymbol{u^\star}=\frac{-\lambda_{\bar{v}_{r} \bar{v}_{\theta}}}{\lambda_{\bar{v}_{r} \bar{v}_{\theta}}}=\frac{p(t)}{p(t)}
$$

where $p(t)=-\lambda_{\bar{v}_r \bar{v}_\theta}$ is the primer vector $\lambda_{\bar{v}_r \bar{v}_\theta}=\sqrt{\lambda_{\bar{v}_r}^2+\lambda_{\bar{v}_\theta}^2}$. Thus, the unit vector $\boldsymbol{u^\star}$ is given by

$$
\boldsymbol{u^\star}=\left[\begin{array}{l}
\frac{-\lambda_{\bar{v}_r}}{\sqrt{\lambda_{\bar{v}_r}^2+\lambda_{\bar{v}_\theta}^2}} \\
\frac{-\lambda_{\bar{v}}}{\sqrt{\lambda_{\bar{v}_r}^2+\lambda_{\bar{v}_\theta}^2}}
\end{array}\right]
$$

The terms in the bracket of the Hamiltonian in Eq. (17) is written as $H^{\prime}$ using $a$ in (9)

$$
\begin{aligned}
H^{\prime} & =-a p-\lambda_{\tilde{m}}\left(\frac{2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right) \\
& =-p\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right)-\lambda_{\tilde{m}}\left(\frac{2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right) \\
& =-\left(\frac{p}{m v_{\theta_{0}}}+\frac{\lambda_{\tilde{m}}}{c m_{0}}\right)\left(\frac{2 \eta P t_{f}}{c}\right)
\end{aligned}
$$

During a constant $I_{s p}$ operation where $c$ is constant, the input power, or equivalently, is the only control. Equation (21) is usually rewritten as

$$
H^{\prime}=-\left(\frac{p}{m v_{\theta_{0}}}+\frac{\lambda_{\tilde{m}}}{c m_{0}}\right) \times\left(T t_{f}\right)=-S_{F} \times\left(T t_{f}\right)
$$

and the sign of the switching function $S_{F}$ determines when the thruster is turned on or off. The choice of the input power, $P$, that minimizes the Hamiltonian in Eq. (21) is then given by the bang-bang control law:

$$
P= \begin{cases}P_{\max } & \text { for } S_{F}>0 \\ 0 & \text { for } S_{F}<0\end{cases}
$$

The maximum available power is adopted when $S_{F}<0$, whereas the engine is switched off when $S_{F}<0$ to minimize $H^{\prime}$, according to the PMP. In addition, the thrust magnitude for $0 \leq T \leq T_{\max }$ will also depend on the algebraic sign of the switching function $S_{F}$.

The Euler-Lagrange equations yield:

$$
\begin{aligned}
& \frac{d \lambda_{\bar{r}}}{d \tau}=-\frac{\partial H}{\partial \bar{r}}=\lambda_{\bar{v}_{r}}\left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}^{2}}-\frac{2}{\bar{r}^{3}}\right) \zeta-\lambda_{\bar{v}_{\theta}} \frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}^{2}} \zeta+\lambda_{\theta}\left(\frac{\bar{v}_{\theta}}{\bar{r}^{2}}\right) \zeta \\
& \frac{d \lambda_{\bar{v}_{r}}}{d \tau}=-\frac{\partial H}{\partial \bar{v}_{r}}=-\lambda_{\bar{r}} \zeta+\lambda_{\bar{v}_{r}} \frac{\bar{v}_{\theta}}{\bar{r}} \\
& \frac{d \lambda_{\bar{v}_{\theta}}}{d \tau}=-\frac{\partial H}{\partial \bar{v}_{\theta}}=-\lambda_{\bar{v}_{r}} \frac{2 \bar{v}_{\theta}}{\bar{r}} \zeta+\lambda_{\bar{v}_{\theta}} \frac{\bar{v}_{r}}{\bar{r}} \zeta-\lambda_{\theta} \frac{1}{\bar{r}} \zeta \\
& \frac{d \lambda_{\theta}}{d \tau}=-\frac{\partial H}{\partial \theta}=0 \\
& \frac{d \lambda_{\tilde{m}}}{d \tau}=-\frac{\partial H}{\partial \tilde{m}}=-\frac{m_{0}}{m^{2}}\left(\frac{2 \eta P t_{f}}{c v_{\theta_{0}}}\right) p
\end{aligned}
$$

From the known initial conditions in Eq. (12) with the known initial time $t_{0}=0 \mathrm{~s}$ and the known final conditions in Eq. (13) with the known final time $t_{f}$, we can get 10 boundary conditions. We apply the transversality condition to pose a well-defined TBPVP.

$$
H_{f} d t_{f}-\lambda_{f}^{T} d y_{f}+d \phi=0
$$

subject to $d \boldsymbol{\Psi}=\mathbf{0}$.

$$
d \boldsymbol{\Psi}=\left[\begin{array}{c}
d r_{f} \\
d v_{r_{f}} \\
d v_{\theta_{f}}
\end{array}\right]=\mathbf{0}
$$

Expanding the nonzero terms in Eq. (26) while noting that we have a Mayer form of the performance index, where $\phi=J=-\tilde{m}_{f}$ :

$$
H_{f} d \tau_{f}-\lambda_{\theta_{f}} d \theta_{f}-\lambda_{\tilde{m}_{f}} d \tilde{m}_{f}-d \tilde{m}_{f}=0
$$

Using the the terminal boundary conditions, we get the following boundary conditions:

$$
\begin{aligned}
& \lambda_{\theta_{f}}=0 \\
& \lambda_{\tilde{m}_{f}}+1=0
\end{aligned}
$$

All ten differential equations and boundary conditions give the well-defined TPBVP:

$$
\begin{aligned}
& \frac{d \bar{r}}{d \tau}=\bar{v}_{r} \zeta,\\
& \frac{d \bar{v}_{r}}{d \tau}=\left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}}-\frac{1}{\bar{r}^{2}}\right) \zeta+\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right)\left(\frac{-\lambda_{\bar{v}_{r}}}{\sqrt{\lambda_{\bar{v}_{\theta}}^{2}+\lambda \bar{v}_{r}}}\right), \\
& \frac{d \bar{v}_{\theta}}{d \tau}=-\frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}} \zeta+\left(\frac{2 \eta P}{m c} \frac{t_{f}}{v_{\theta_{0}}}\right)\left(\frac{-\lambda_{\bar{v}_{\theta}}}{\sqrt{\lambda_{\bar{v}_{\theta}}^{2}+\lambda_{\bar{v}_{r}}^{2}}}\right), \\
& \frac{d \theta}{d \tau}=\frac{\bar{v}_{\theta}}{\bar{r}} \zeta, \\
& \frac{d \tilde{m}}{d \tau}=\left(\frac{-2 \eta P}{c^{2}} \frac{t_{f}}{m_{0}}\right), \\
& \frac{d \lambda_{\bar{r}}}{d \tau}=\lambda_{\bar{v}_{r}}\left(\frac{\bar{v}_{\theta}^{2}}{\bar{r}}-\frac{2}{\bar{r}^{3}}\right) \zeta-\lambda_{\bar{v}_{\theta}} \frac{\bar{v}_{r} \bar{v}_{\theta}}{\bar{r}^{2}} \zeta+\lambda_{\theta}\left(\frac{\bar{v}_{\theta}}{\bar{r}^{2}}\right) \zeta,\\
& \frac{d \lambda_{\bar{v}_r}}{d \tau}=-\lambda_{\bar{r}} \zeta+\lambda_{\bar{v}_\theta} \frac{\bar{v}_\theta}{\bar{r}},\\
& \frac{d \lambda_{\bar{v}_{\theta}}}{d \tau}=-\lambda_{\bar{v}_{r}} \frac{2 \bar{v}_{\theta}}{\bar{r}} \zeta+\lambda_{\bar{v}_{\theta}} \frac{\bar{v}_{\theta}}{\bar{r}}-\lambda_{\theta} \frac{1}{\bar{r}} \zeta, \\
& \frac{d \lambda_{\theta}}{d \tau}=0, \\
& \frac{d \lambda_{\tilde{m}}}{d \tau}=\frac{m_{0}}{m^{2}}\left(\frac{2 \eta P}{c} \frac{t_{f}}{v_{\theta_{0}}}\right)(p-1) \\
\end{aligned}
$$

The 10 boundary conditions are given with initial $\tau_0 = 0$ to get the following initial conditions:

$$
\bar{r}_{0}=1, \\
\bar{v}_{r_{0}}=0, \\
\bar{v}_{0}=1, \\
\theta_{0}=0, \\
\tilde{m}_{0}=1 \\
$$

and final $\tau_f = 1$ to get the following final conditions:

$$
\bar{r}_{f}=\frac{r_{f}}{r_{0}}, \\
\bar{v}_{r_{f}}=0, \\
\bar{v}_{f}=\sqrt{\frac{1}{\bar{r}_{f}}}, \\
\lambda_{\theta_{f}}=0, \\
\lambda_{\tilde{m}_{f}}+1=0
$$

## Direct Method

The same minimum-fuel problem is solve with the direct method. Minimize the performance index in Eq. (15) subject to the dynamics constraints which is the scaled equation of motion in Eq. (11) The continuous-time state variables $\boldsymbol{y}$ and control variables $\boldsymbol{u}$ are given, respectively as

$$
\boldsymbol{y}(t)=\left[\bar{r}(t),\ \bar{v}_r(t),\ \bar{v}_\theta(t),\ \theta(t),\ \tilde{m}(t)\right] \in \mathbb{R}^5,
$$

$$
\boldsymbol{u}(t)=\left[u_1(t),\ u_2(t),\ P\right] \in \mathbb{R}^3
$$

The equality path constraint function

$$
e=u_{1}^{2}(t)+u_{2}^{2}(t)=1
$$

and boundary condition function are given as

$$
\begin{aligned}
&b_{1}=\bar{r}_{0}-1=0,\\
&b_{2}=\bar{v}_{r_{0}}=0,\\
&b_{3}=\bar{v}_{0}-1=0,\\
&b_{4}=\theta_{0}=0,\\
&b_{5}=\tilde{m}_{0}, \\
&b_{5}=\bar{r}_{f}-\frac{r_{f}}{r_{0}}=0,\\
&b_{6}=\bar{v}_{r_{f}}=0, \\
&b_{7}=\bar{v}_{f}=\sqrt{\frac{1}{\bar{r}_{f}}}
\end{aligned}
$$

The right-hand side function of the dynamics, the path constraint function, and the boundary functions are written, respectively, as

$$
\boldsymbol{a}(\boldsymbol{y}(t), \boldsymbol{u}(t), t)=\left[\begin{array}{c}
\bar{v}_r \\
\frac{\bar{v}_\theta^2}{\bar{r}}-\frac{1}{\bar{r}^2}+a u_1 \\
-\frac{\bar{v}_r \bar{v}_\theta}{\bar{r}_\theta}+a u_2 \\
\frac{\bar{v}_{\theta}}{\bar{r}}\\
\dot{\tilde{m}}
\end{array}\right]
$$

$$
\boldsymbol{e}(\boldsymbol{y}(t), \boldsymbol{u}(t), t)=u_{1}^{2}(t)+u_{2}^{2}(t)-1
$$

$$
\begin{aligned}
\boldsymbol{b}\left(\boldsymbol{y}\left(t_{0}\right), t_{0} \boldsymbol{y}\left(t_{f}\right), t_{f}\right)=\left[\begin{array}{c}
&\bar{r}_{0}-1 \\
&\bar{v}_{r_{0}} \\
&\bar{v}_{0}-1 \\
&\theta_{0} \\
&\tilde{m}_{0}-1 \\
&\bar{r}_{f}-\frac{r_{f}}{r_{0}} \\
&\bar{v}_{r_{f}} \\
&\bar{v}_{f}-\sqrt{\frac{1}{\bar{r}_{f}}}
\end{array}\right]
\end{aligned}
$$

Finally, the lower and upper bounds on the path constraints and boundary conditions are all zero. Finally, the lower and upper bounds on the path constraints and boundary conditions are all zero. Because the first five boundary conditions, $\left(b_{1}, \ldots, b_{7}\right)$, are simple bounds on the initial and final continuous-time state, they will be enforced in the NLP as simple bounds on the NLP variables corresponding to the initial and terminal state. The 7 th boundary condition, $b_{7}$, on the other hand, is a nonlinear function of the terminal state and, thus, will be enforced in the NLP as a nonlinear constraint.

# Numerical Results

In this section, the mission scenarios of the planar Earth-to-Mars orbit transfer at five different input powers and the flight time are given to validate minimum-fuel low-thrust trajectory optimization with the constant specific impulse engines. Both the indirect and direct methods are used to solve the same problem and their solutions are compared. The initial and final orbital parameters are displayed in Table 1. It is observed that the final position in the mission orbit (Mars orbit) is not specified since the purpose of the simulation is to reach the orbit not the planet. As the forces acting on the spacecraft, the Sun's gravity and the thrust produced by the engines are considered. The spacecraft is assumed to have initial mass $1500 \mathrm{~kg}$ and the CSI engines with the $I_{s p}=3300 \mathrm{~s}$ and the constant thruster efficiency, $\eta=0.7$. The used input powers and the corresponding flight time are listed in Table 2. The input powers and flight time conditions of the scenario 1, 2 and 3 are listed in Table 2. The scenario 1 has $19 \mathrm{kw}$ of input power and 240 days of flight time, the scenario 1 has $7.5 \mathrm{kw}$ of input power and 365 days of flight time, and the scenario 3 has $3.6 \mathrm{kw}$ of input power and 2 years of flight time. In these scenarios, the indirect method solutions are obtained by solving the well-defined TPBVP in Eq. (29) with bvp4c function in MATLAB while the direct method solutions are obtained with GPOPS-II in the Section 3.2. In addition, SNOPT [15] is used as a NLP solver in GPOPS-II.

Table 1. Intial (Earth) and tareget (Mars) orbits to test the optimization methods
| | Initial orbit: Earth | Target orbit: Mars |
|-----------------------------------------------|----------------------|--------------------|
| Radius, $\mathrm{r}(\mathrm{AU})$ | 1 | 1.525588896382880 |
| Orbital speed, $v(\mathrm{~km} / \mathrm{s})$ | 29.7847 | 29.494 |
| Location, $\theta(\mathrm{deg})$ | 0 | - |

Table 2. Input powers $(P)$ and flight time $\left(t_{f}\right)$

| Scenario   | Input power $(\mathbf{k W})$ | Flight time (days) |
| ---------- | ---------------------------- | ------------------ |
| Scenario 1 | 19                           | 240                |
| Scenario 2 | 7.5                          | 365                |
| Scenario 3 | 3.6                          | $2 \times 365$     |

Figures 2, 3, and 4 present the results of a transfer from Earth to Mars orbit in the heliocentric reference frame in astronomical units (AU) using the indirect and direct methods for the scenarios 1 , 2 and 3, respectively. The dashed inner circle represents the Earth's orbit while the dashed outer circle represents the Mars' orbit. The curves represent the spacecraft trajectory while the arrows represent the thrust unit vectors. Since the indirect method uses 10000 mesh points, the thrust unit vectors in the Earth-to-Mars trajectories are not obviously indistinguishable. On the other hand, the direct method obvious thrust unit vectors along the trajectories because the number of nodes adjusted in the the optimization calculation. However, the Earth-to-Mars trajectories using the indirect method and direct methods are matching with no remarkable difference. These numerically matching results are verified in the control acceleration and the thrust angle as shown in Figs. 5, 6 and 7. In these results, the control accelerations by the thrust on-off-on thrusting sequences are obtained by the ban-bang control law (23) for the CSI engines. The thrust on-off-on thrusting sequences are also analyzed using the primer vector theory, which appear in the next figures. The control accelerations and thrust angle $(\alpha)$ are also almost matching except the thrust off phases. It is due to some numerical difference in the indirect and direct methods. However, the numerical differences in the results are not remarkable enough to result in different optimization results. Figures 8 and 9 present time histories of thrust and acceleration magnitudes, and time histories of the propellant mass and total mass using the indirect and direct methods for scenario 1. Numerically matching results are also obtained for scenario 1. Table 3 shows the $\Delta v$ and the propellant masses for scenario 1, 2 and 3, respectively using the indirect and direct methods. The numerical integration of the control acceleration is computed as $\Delta v$ using $\int_{0}^{t_{f}}(T / m) d t$. They are almost numerically matching in both methods. The propellant mass $m_{p}$ are almost numerically matching in both methods. As the flight time increases in the scenario 1, 2 and 3 , Deltav and the propellant mass $\left(m_{p}\right)$ get smaller. Thus, the fuel consumption could be more saved in the longer flight time and smaller input power.

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-10.jpg?height=500&width=623&top_left_y=344&top_left_x=588)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-10.jpg?height=500&width=623&top_left_y=338&top_left_x=1276)

(b)

Figure 2. Planar Earth-to-Mars low-thrust trajectory for the scenario 1, (a) Indirect method, (b) Direct method

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-10.jpg?height=485&width=623&top_left_y=1071&top_left_x=588)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-10.jpg?height=485&width=623&top_left_y=1071&top_left_x=1276)

(b)

Figure 3. Planar Earth-to-Mars low-thrust trajectory for the scenario 2, (a) Indirect method, (b) Direct method

Table 3. Comparisons of the optimization solutions for scenario 1, 2 and 3.

| Scenario   | Indirect Method$\Delta v(\mathrm{~km} / \mathrm{s})$ | Direct Method $\Delta v(\mathrm{~km} / \mathrm{s})$ | Indirect Method $m_{p}(\mathrm{~kg})$ | Direct Method $m_{p}(\mathrm{~kg})$ |
| ---------- | :--------------------------------------------------: | :-------------------------------------------------: | :-----------------------------------: | :---------------------------------: |
| Scenario 1 |                        9.470                         |                        9.469                        |                380.558                |               380.559               |
| Scenario 2 |                        7.001                         |                        7.005                        |                292.010                |               292.028               |
| Scenario 3 |                        5.694                         |                        5.693                        |                241.970                |               241.970               |

Figures 10, 11 and 12 present the switching functions and thrust profiles for scenario 1, 2 and 3, respectively. They show on-off-on thrusting sequences corresponding to the bang-bang control law in (23). The thrust magnitude switches its limiting values of 0 (null-thrust arc) and $T_{\max }$ (maximum-thrust arc) each time $S_{F}(t)$ passes through 0 according to Eq. (22). The left side of Figs. 10, 11 and 12 are obtained in the indirect method while the right side of them are obtained in GPOPS-II. In general, the costate vectors are computed in indirect method. However, GPOP-II can provide the computed costate vectors in the optimization algorithm. With these costate vectors, the switching function $S_{F}$ and the thrust magnitude $(T)$ are also obtained in the right side of Figs. 10, 11 and 12. The switching functions and thrust profiles using the indirect and direct methods for scenario 1, 2 and 3 present matching results. Thus, minimum-fuel planar Earth-to Mars low-thrust trajectories using bang-bang control for the CSI engines have been validated by both the indirect method and direct method. In addition, it has been demonstrated that $\Delta v$ and the propellant mass $m_{p}$ can be saved in longer flight time and smaller input power through three selected mission scenarios.

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-11.jpg?height=502&width=623&top_left_y=340&top_left_x=588)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-11.jpg?height=497&width=623&top_left_y=343&top_left_x=1276)

(b)

Figure 4. Planar Earth-to-Mars low-thrust trajectory for the scenario 3, (a) Indirect method, (b) Direct method
![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-11.jpg?height=242&width=1334&top_left_y=1038&top_left_x=592)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-11.jpg?height=239&width=625&top_left_y=1314&top_left_x=590)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-11.jpg?height=248&width=646&top_left_y=1304&top_left_x=1276)

(b)

Figure 5. Time history of control variables for scenario 1, (a) Indirect method, (b) Direct method

# Conclusion

In this paper, minimum-fuel planar heliocentric Earth-to-Mars low-thrust trajectories using bang-bang control for constant specific impulse engines of spacecraft have been studied. For the problem formulation, the derivation of the scaled equations of motion is described in detail. A well-defined TPBVP is derived for the indirect method while GPOPS-II is emoyed to solve the problem. Using the three selected mission scenarios with different flight time and input powers, minimum-fuel planar low-thrust trajectories with on-off-on thrust sequences are validated with both the indirect and direct methods. Numerically matching optimization results are obtained by both methods with the same on-off-on thrust sequence. The results are useful for broad trajectory search using the CSI engine in the preliminary phase of mission designs.
![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=244&width=1330&top_left_y=370&top_left_x=615)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=251&width=626&top_left_y=640&top_left_x=612)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=251&width=646&top_left_y=640&top_left_x=1299)

(b)

Figure 6. Time history of control variables for scenario 2, (a) Indirect method, (b) Direct method
![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=250&width=1334&top_left_y=1188&top_left_x=610)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=254&width=642&top_left_y=1461&top_left_x=593)

$(\mathrm{a})$

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=251&width=646&top_left_y=1462&top_left_x=1299)

(b)

Figure 7. Time history of control variables for scenario 3, (a) Indirect method, (b) Direct method
![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-12.jpg?height=546&width=1408&top_left_y=1993&top_left_x=608)

Figure 8. Time histories of thrust and acceleration magni-Figure 9. Time histories of propellant masses and timetudes for scenario 1 varying masses for scenario 1 .

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-13.jpg?height=525&width=699&top_left_y=363&top_left_x=587)

$(\mathrm{a})$

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-13.jpg?height=523&width=711&top_left_y=361&top_left_x=1278)

(b)

Figure 10. CSI switching functions and thrust profiles for the scenario 1, (a) Indirect method, (b) GPOPS-II

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-13.jpg?height=528&width=711&top_left_y=1164&top_left_x=570)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-13.jpg?height=528&width=713&top_left_y=1164&top_left_x=1254)

(b)

Figure 11. CSI switching functions and thrust profiles for the scenario 2, (a) Indirect method, (b) GPOPS-II

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-13.jpg?height=534&width=719&top_left_y=1960&top_left_x=566)

(a)

![](https://cdn.mathpix.com/cropped/2023_03_16_02e6e1e6d3ee7283f8aeg-13.jpg?height=531&width=696&top_left_y=1962&top_left_x=1274)

(b)

Figure 12. CSI switching functions and thrust profiles for the scenario 3, (a) Indirect method, (b) GPOPS-II

# References

1. Rayman, M.D.; Williams, S.N. Design of the first interplanetary solar electric propulsion Mission. Journal of Spacecraft and Rockets 2002, $39,589-595$.

2. Rayman, M.D.; Fraschettia, T.C.; Raymonda, C.A.; Russell, C.T. Dawn: a mission in development for exploration of main belt asteroids Vesta and Ceres. Acta Astronautica 2006, 58, 605-616.

3. Kugelberg, J.; Bodin, P.; Persson, S.; Rathsman, P. Accommodating electric propulsion on SMART-1. Acta Astronautica 2004, 55, 121-130.

4. Nishiyama, B.K.; Hosoa, S.; Ueno, K.; Tsukizaki, R.; Kuninaka, H. Development and Testing of the Hayabusa2 Ion Engine System. Trans. JSASS Aerospace Tech. Japan 2016, 14, 131-140.

5. Chadalavada, P.; Farabi, T.; Dutta, A. Sequential Low-Thrust Orbit-Raising of All-Electric Satellites. Aerospace 2020, 7, 1-27.

6. Betts, J.T. Survey of numerical methods for trajectory optimization. Journal of Guidance, Control and Dynamics $1998,21,193-207$.

7. Morante, D.; Rivo, M.S.; Soler, M. A Survey on Low-Thrust Trajectory Optimization Approaches. Aerospace 2021, 8, 1-39.

8. Abdelkhalik, O. Dynamic-size multiple populations genetic algorithm for multigravity-assist trajectory optimization. Journal of Guidance, Control and Dynamics 2012, 35, 520-529.

9. Shirazi, A. Analysis of a hybrid genetic simulated annealing strategy applied in multi-objective optimization of orbital maneuvers. IEEE Aerospace and Electronic Systems Magazine 2017, 32, 6-22.

10. Lawden, D.F. Optimal Trajectories for Space Navigation. Butterworths, 1963, pp. 54-60.

11. Longuski, J.M.; Guzmán, J.J.; Prussing, J.E. Optimal Control with Aerospace Applications. Springer, 2014, pp. 197-198.

12. Patterson, M.A.; Rao, A.V. GPOPS-II: A MATLAB Software for Solving Multiple-Phase Optimal Control Problems Using hp-Adaptive Gaussian Quadrature Collocation Methods and Sparse Nonlinear Programming. ACM Transactions on Mathematical Software 2014, 41, 1-37.

13. Topputo, F.; Zhang, C. Survey of direct transcription for low-thrust space trajectory optimization with applications. Abstract and Applied Analysis 2014, 2014, 1-15.

14. Conway, B.A. Spacecraft Trajectory Optimization. Cambridge, 2010, pp. 16-24, 49-62.

15. Gill, P.E.; Murray, W.; Saunders, M.A. SNOPT: An SQP Algorithm for Large-Scale Constrained Optimization. SIAM Review 2005, $47,99-131$.
