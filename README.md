# Enhancing Tractor-Trailer System Stability through Nonlinear Anti-Jackknifing Model Predictive Control

This repository contains the code and [documentation](facci_matteo_master_thesis.pdf) for my master's thesis titled "Enhancing Tractor-Trailer System Stability through Nonlinear Anti-Jackknifing Model Predictive Control." In this thesis, we propose a control strategy to prevent the jackknife effect in tractor-trailer systems using a nonlinear Model Predictive Control (MPC) approach.

#### Key Features
- Implementation of a nonlinear MPC approach for trajectory tracking and anti-jackknifing control
- Comparison of basic and advanced stabilizing terminal constraints for improved performance
- Simulation results demonstrating the effectiveness of the proposed control technique
- Integration of obstacle avoidance capabilities using the Rapidly Exploring Random Tree* (RRT*) algorithm

### Introduction

The jackknife effect occurs when the angle between the tractor and the trailer diverges, leading to dangerous accidents. Our objective is to prevent this effect by utilizing a nonlinear MPC. This README provides a summary of the key points discussed in the thesis.

### Problem Formulation and Implementation

This is a standard trajectory tracking, where the goal is to minimize the error between a given reference trajectory (cartesian coordinates) and the actual trajectory. We observe that the system exhibits instability with negative velocities during reverse motion due to the unstable zero dynamics of the angular variables. To address this, we introduce a kinematic model for the tractor-trailer system, making certain assumptions about rigid bodies, control points, road conditions, and input characteristics.

### Nonlinear MPC Formulation

To solve the trajectory tracking problem, we employ a Model Predictive Control (MPC) approach. The discrete version of the MPC involves computing an optimal control sequence over a prediction horizon. We solve an optimal control problem to find the best control sequence. Stabilizing terminal constraints are introduced to mitigate the instability problem, utilizing auxiliary trajectories obtained through output error feedback linearization.

### Output Error Feedback Linearization

Output error feedback linearization is a control technique employed in trajectory tracking. The reference trajectory and its derivative are used as control inputs, transformed by the inverse of a decoupling matrix to obtain a decoupled system. This technique enables stable forward motion, which is crucial for trajectory tracking and for obtaining a feasible auxiliary trajectory and stabilizing terminal constraints.

### Stabilizing Terminal Constraints

Stabilizing terminal constraints are incorporated to enhance system stability. Two versions of stabilizing terminal constraints are considered: basic and advanced. The basic version uses an equality constraint on the unstable variables between the state and the auxiliary trajectory. The advanced version takes into account all state variables by expressing the system dynamics in terms of error coordinates and linearizing them around the auxiliary trajectory. A transformation matrix is used to decouple the state matrix, resulting in a new terminal constraint.

### Implementation of the Proposed Approach

The proposed approach is implemented using numerical optimization techniques. We transcribe the optimal control problem into a Multiple Shooting Nonlinear Programming Problem and utilize the Runge Kutta 4th order integration method. The simulations are conducted in MATLAB using the [CasADi Toolbox](https://github.com/casadi), known for its capabilities in numerical optimization and NLP problems.

### Simulations and Results

Several simulations are performed to evaluate the effectiveness of the proposed approach. The advanced stabilizing terminal constraint consistently outperforms the basic constraint in terms of tracking accuracy, control horizon, iteration time, and settling time. Various reference trajectories are tested, including linear, circular, rectangular, and obstacle avoidance paths. The simulations demonstrate successful trajectory tracking and system stability.

### State-of-the-Art and Future Work

The thesis discusses the state-of-the-art control scheme, [An Intrinsically Stable MPC Approach for Anti-Jackknifing Control of Tractor-Trailer Vehicles](https://ieeexplore.ieee.org/document/9740173). A comparison between the proposed Nonlinear MPC approach and the state-of-the-art method is provided, highlighting the advantages of the Nonlinear MPC in handling tracking and stabilization simultaneously. Future work includes validating the results with an experimental prototype, exploring online obstacle avoidance strategies, and enhancing the existing Rapidly Exploring Random Tree (RRT) algorithm by incorporating techniques such as Bezier curves and different motion primitives.

### Conclusion

The proposed Nonlinear Anti-Jackknifing Model Predictive Control approach effectively prevents the jackknife effect in tractor-trailer systems by ensuring stable trajectory tracking. The advanced stabilizing terminal constraint outperforms the basic constraint in terms of tracking accuracy and convergence speed. The simulations demonstrate the robustness and versatility of the proposed approach, making it a promising solution for real-world applications.

For more details, please refer to the complete thesis [documentation](facci_matteo_master_thesis.pdf) and code in this repository.

## Dependencies
The code in this repository relies on the following dependencies:
- MATLAB
- [CasADi Toolbox](https://github.com/casadi)

Make sure you have the necessary software and libraries installed before running the simulations or modifying the code.

## :warning: Important Note :warning:

**Make sure to consider the following before using this repository:**

This project has been developed and tested on **Windows 11** using a specific version of the CasADi Toolbox. Please note that if you are using a different operating system or a different version of CasADi, there might be compatibility issues or differences in the toolbox functionality.

If you are using a different operating system, you may need to adapt the code or obtain the appropriate version of CasADi for your system.

It is highly recommended to review the documentation and any system requirements mentioned to avoid any potential conflicts or errors during the setup and execution of the code.

Thank you for your understanding!


## License
Feel free to use the code and documentation for academic or personal purposes, but please acknowledge the original work by citing the thesis appropriately.

## Acknowledgments
I would like to express my gratitude to my thesis advisor and co-advisor for their guidance and support throughout this project. Their valuable insights and feedback have greatly contributed to the development of this control technique.


For any further questions or inquiries, please feel free to contact me at [matteo.facci@outlook.it]
