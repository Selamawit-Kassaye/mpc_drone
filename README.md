# acm_mpc_drones

# Drone Control: A Non-Linear Model Predictive Control approach for Trajectory Tracking and Obstacle Avoidance

## Table of Contents
1. [Overview](#overview)
2. [Problem Statement](#problem_statement)
3. [Results](#results)
4. [Running the Project](#running_project)
5. [Appendix](#appendix)
6. [Bibliography](#bibliography)
7. [Directory Structure](#directory_struct)

## Overview <a id="overview"></a>
The project focuses on implementing a robust and efficient takeoff procedure for the Crazyflie quadrotor using Model Predictive Control (MPC). MPC is a powerful control strategy that optimizes control inputs over a future time horizon while considering system dynamics and constraints. This approach ensures that the drone can take off smoothly and safely while maintaining stability and adhering to predefined constraints.

**Team Members:**
Lavanya Ratnabala 
Selamawit Asfaw


**Presentation:** [From Ground to Air: Crazyflie Drone Takeoff with Model Predictive Control](https://docs.google.com/presentation/d/1lm82zUs13wzlbzYoQ151rFkZ4N4f6DnRfFaLImc0srU/edit#slide=id.p4)

## Motivation
In the last decade, drones have gained immense popularity, revolutionizing sectors like agriculture, surveillance, delivery, and entertainment with their efficiency, precision, and automation. However  takeoff phase involves complex dynamics due to the interactions between the UAV’s propulsion system and environmental factors like wind and turbulence.  Improper takeoff can lead to crashes or unstable flight paths, posing safety risks to nearby people and structures and potential mission failure.

## Problem Statement and solution <a id="problem_statement and solution"></a>
Traditional control methods like PID controllers often struggle with the precision and proactive adjustments required during the takeoff phase of drone flights, particularly in handling dynamic external forces such as wind and turbulence. These forces can introduce instabilities that reactive control systems cannot effectively anticipate, leading to potential safety risks and inefficiencies. Implementing Model Predictive Control (MPC) for the Crazyflie drone's takeoff phase enhances stability and accuracy by using a dynamic model to predict future states and adjust controls in real-time. This proactive approach allows the system to anticipate and mitigate the effects of external disturbances, optimizing control inputs within the drone's operational constraints. Consequently, MPC ensures a safer, more reliable, and efficient takeoff, overcoming the limitations of traditional reactive control methods.

## Results <a id="results"></a>
The implemented MPC controller successfully worked on real drones using motion capture and VICON tracking stabilizes the drone at the desired takeoff point:

- By leveraging MPC, the drone is able to proactively predict and achive the takesoff.
  Additionally, the use of MPC reduces the occurrence of instabilities and enhances the safety and reliability of the takeoff 
  process.
- Empirical data shows a marked small deviation from the planned takeoff path and a more consistent achievement of desired altitude and orientation targets.




### Visual Aids
The following plots illustrate the performance of the MPC controller:

1. **MPC Response for point stabilization with terminal cost**
   
   ![Point Stabilization for N = 6 with terminal cost](results/point_stable_n_6_tp.gif)

2. **Trajectory Tracking to pass through gates**
   
   ![Passing through Gates trajectory](results/trajectory.jpeg)
   
3. **Obstalce Avoidance (2D View)**

   ![Obstacle Avoidance](results/drone_obs_avoid.gif)

## Running the Project <a id="running_project"></a>
Follow these steps to set up and run the project on your local machine.

### Prerequisites
- Python 3.x
- CasADi
- NumPy
- Matplotlib

### Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/R-Ahmed-Khan/acm_mpc_drones.git
    cd acm_mpc_drones
    ```

2. Install the required packages:
    ```bash
    pip install -r requirements.txt
    ```

### Running the Code
1. **Single Point Stabilization:**
    ```bash
    python mpc_drone_single_point.py
    ```

2. **Trajectory Tracking:**
    ```bash
    python mpc_drone_trajectory.py
    ```

3. **Obstacle Avoidance:**
    ```bash
    python mpc_drone_obstacle_avoid.py
    ```

### System Dynamics
# State Variiables
    - Position 
    - Velocity
    - Altitude

# Control Inputs
    - thrust and torques




#### Control and Dynamical parameters
```bash
...
# Quadrotor Parameters
    mass = 0.027#0.028
    arm_length= 0.0397#0.044
    Ixx= 1.4e-5#2.3951e-5
    Iyy=1.4e-5#2.3951e-5
    Izz=2.17e-5#3.2347e-5
    cm=2.4e-6
    tau= 0.08

# MPC Parameters
    mpc_tf = 1.0
    mpc_N = 50
    control_update_rate = 50
    plot_trajectory = True
                                 # scalling factor
...
```
#### Cost Weights
```bash
...
   Q = np.diag([20., 20., 20., 2., 2., 2., 1., 1., 1.])   # state weighting matrix
   R = diag(horzcat(1., 1., 1., 1.))                      # control input weighting matrix.
   W = block_diag(Q,R)                                    # Combined Weighting Matrix 
...
```


## Appendix <a id="appendix"></a>

### Quadrotor Dynamics

The quadrotor's dynamics are described by the following state-space equations, where the state vector includes positions, orientations, and their respective velocities, and the control inputs are the propeller speeds.

#### States

- Positions in the inertial frame: $$\( x, y, z \)$$ 
- Roll, pitch, and yaw angles: $$\( \phi, \theta, \psi \)$$ 
- Velocities in the inertial frame: $$\( \dot{x}, \dot{y}, \dot{z} \)$$ 
- Angular velocities: $$\( \dot{\phi}, \dot{\theta}, \dot{\psi} \)$$



# Setup explicit ode equations
\[\dot{px} = vx\]
\[\dot{py} = vy\]
\[\dot{pz} = vz\]
\[\dot{vx} = vdot[0]\]
\[\dot{vy} = vdot[1]\]
\[\dot{vz} = vdot[2]\]
\[\dot{roll} = \frac{(roll_c - roll)}{\tau}\]
\[\dot{pitch} = \frac{(pitch_c - pitch)}{\tau}\]
\[\dot{yaw} = \frac{(yaw_c - yaw)}{\tau}\]
# vector function of explicit dynamics
\[f_{expl} = \begin{bmatrix} \dot{px} \\ \dot{py} \\ \dot{pz} \\ \dot{vx} \\ \dot{vy} \\ \dot{vz} \\ \dot{roll} \\ \dot{pitch} \\ \dot{yaw} \end{bmatrix}\]


#### Inputs
- : Propeller speeds: $$\( u_1, u_2, u_3, u_4 \)$$

#### Equations of Motion

1. **Position Dynamics:**  
   $$\dot{x} = v_x $$
   $$\dot{y} = v_y $$
   $$\dot{z} = v_z $$


2. **Orientation Dynamics:**
   $$\dot{\phi} = \omega_x $$
   $$\dot{\theta} = \omega_y $$
   $$\dot{\psi} = \omega_z $$

3. **Velocity Dynamics:**

   $$\dot{v_x} = \frac{1}{m}\left(\cos(\phi) \sin(\theta) \cos(\psi) + \sin(\phi) \sin(\psi)\right) f_1 - \frac{K_1 v_x}{m}$$
   $$\dot{v_y} = \frac{1}{m}\left(\cos(\phi) \sin(\theta) \sin(\psi) - \sin(\phi) \cos(\psi)\right) f_1 - \frac{K_2 v_y}{m}$$
   $$\dot{v_z} = -\frac{1}{m}\left(\cos(\psi) \cos(\theta)\right) f_1 + g - \frac{K_3 v_z}{m}$$


4. **Angular Velocity Dynamics:**
   $$\dot{\omega_x} = \frac{I_y - I_z}{I_x} \omega_y \omega_z + \frac{l}{I_x} f_2 - \frac{K_4 l}{I_x} \omega_x + \frac{J_r}{I_x} \omega_y (u_1 - u_2 + u_3 - u_4)$$
   $$\dot{\omega_y} = \frac{I_z - I_x}{I_y} \omega_x \omega_z + \frac{l}{I_y} f_3 - \frac{K_5 l}{I_y} \omega_y + \frac{J_r}{I_y} \omega_z (u_1 - u_2 + u_3 - u_4)$$
   $$\dot{\omega_z} = \frac{I_x - I_y}{I_z} \omega_x \omega_y + \frac{1}{I_z} f_4 - \frac{K_6}{I_z} \omega_z$$

#### Forces and Torques
- Thrust: $$f_1 = b(u_1^2 + u_2^2 + u_3^2 + u_4^2) $$
- Roll Moment: $$f_2 = b(-u_2^2 + u_4^2) $$
- Pitch Moment: $$f_3 = b(u_1^2 - u_3^2) $$
- Yaw Moment: $$f_4 = d(-u_1^2 + u_2^2 - u_3^2 + u_4^2) $$

### State and Input Constraints
#### State Constraints
- Position: $$-\infty \leq x,y,z \leq +\infty$$
- Orientation: $$\( -\pi/2 \leq \phi, \theta, \psi \leq \pi/2 \)$$
- Velocities: $$\( -5 \, \text{m/s} \leq v_x, v_y, v_z \leq 5 \, \text{m/s} \)$$
- Angular Velocities: $$\( -0.05 \, \text{rad/s} \leq \omega_x, \omega_y, \omega_z \leq 0.05 \, \text{rad/s} \)$$

#### Input Constraints
- Propeller Speeds: $$\( 0 \leq u_1, u_2, u_3, u_4 \leq 1200 \)$$

### Cost Function 

The cost function is designed to minimize the error between the current state and the desired state while penalizing large control inputs. It consists of a quadratic term for state deviation and another for control effort.

$$Cost = \sum_{i=0}^{N-1} ((x_i - x_p)^T Q (x_i - x_p) + u_i^T R u_i) + (x_N - x_p)^T Q_{terminal} (x_N - x_p)$$
where $$x_p$$ is desired state, $$x_i$$ is current state and $$x_N$$ is terminal state

## Bibliography <a id="bibliography"></a>

1. M. Islam, M. Okasha, and M. M. Idres, “Dynamics and control of quadcopter using linear model predictive control approach,” IOP Conf. Ser. Mater. Sci. Eng., vol. 270, p. 012007, Dec. 2017, doi: 10.1088/1757-899X/270/1/012007.
2. F. Adıgüzel and T. V. Mumcu, “Discrete-time Backstepping Attitude Control of a Quadrotor UAV,” in 2019 International Artificial Intelligence and Data Processing Symposium (IDAP), Sep. 2019, pp. 1–5. doi: 10.1109/IDAP.2019.8875891.
3. “Nonlinear Model Predictive Control of a Quadrotor.” Accessed: May 27, 2024. [Online]. Available: https://upcommons.upc.edu/handle/2117/98503
4. “Applied Sciences | Free Full-Text | A Comparative Study for Control of Quadrotor UAVs.” Accessed: May 27, 2024. [Online]. Available: https://www.mdpi.com/2076-3417/13/6/3464
5. “(2) MPC and MHE implementation in Matlab using Casadi - YouTube.” Accessed: May 27, 2024. [Online]. Available: https://www.youtube.com/playlist?list=PLK8squHT_Uzej3UCUHjtOtm5X7pMFSgAL
6. M. W. Mehrez, “MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi.” May 19, 2024. Accessed: May 27, 2024. [Online]. Available: https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi
7.	“Energies | Free Full-Text | Non-Linear Model Predictive Control Using CasADi Package for Trajectory Tracking of Quadrotor.” Accessed: May 27, 2024. [Online]. Available: https://www.mdpi.com/1996-1073/16/5/2143

## Directory Structure <a id="directory_struct"></a>
- `mpc_drone_single_point.py`: Script for single point stabilization.
- `mpc_drone_trajectory.py`: Script for trajectory tracking.
- `mpc_drone_obstacle_avoid.py`: Script for obstacle avoidance.
- `resources/draw.py`: Helper functions for plotting results.
- `resources/trajectory.py`: Helper functions for trajectory generation.
- `results/`: Directory containing generated plots for visualization.

Feel free to utilize this repository for your projects :)
