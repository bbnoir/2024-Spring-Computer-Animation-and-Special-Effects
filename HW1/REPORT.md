# HW1 REPORT

110511010 楊育陞

## Introduction

In this homework, we implement particle systems with different integrators and parameters. Using these particle systems, we simulate the motion of cloth and collision with a sphere. Furthermore, by comparing the results of different integrators and parameters, we analyze the effect of them on the simulation. Finally, I implement a bonus feature to constrain some particles of the cloth.

## Fundamentals

### 1. Structure of Cloth in Particle System

The structure of cloth is a 2D grid of particles, with each particle connected to its neighbors by springs. The springs are divided into three types:

1. **Structural Springs**: connect the particle to its neighbors in the same row and column.
2. **Shear Springs**: connect the particle to its diagonal neighbors.
3. **Bend Springs**: connect the particle to its neighbors in the same row and column with a distance of 2.

<div align="center">
    <img src="img/cloth_struct.png" width="50%">
</div>


### 2. Internal Forces

There are two types of internal forces in the cloth:

1. **Spring Force**: The force exerted by the springs connecting the particles. In this homework, the spring force is calculated by following formula, where $k_{spring}$ is the parameter `springCoef`:

$$
F_{spring} = -k_{spring} \cdot (|x_{a} - x_{b}| - l_{0}) \cdot \frac{{x_{a} - x_{b}}}{{|x_{a} - x_{b}|}}
$$

2. **Damping Force**: The force that resists the motion of the particles. In this homework, the damping force is calculated by the following formula, where $k_{damping}$ is the parameter `damperCoef`:

$$
F_{damping} = -k_{damping} \cdot \left(\frac{(v_{a}-v_{b}) \cdot (x_{a}-x_{b})}{|x_{a}-x_{b}|}\right) \frac{x_{a}-x_{b}}{|x_{a}-x_{b}|}
$$

### 3. Collision

In this homework, the collision is detected by the distance between the particle and the sphere, also the velocity of the particle. If collision happends, the particle will be applied with a collision force, which is calculated by the following formula:

$$
v_{a} = \frac{{m_{a}u_{a} + m_{b}u_{b} - m_{b}C_{R}(u_{a} - u_{b})}}{{m_{a} + m_{b}}}
$$

We only consider the normal direction of the collision, also we assume the sphere is fixed.

### 4. Integrators

1. **Explicit Euler**: The simplest integrator, which is easy to implement but not stable for large time steps.
2. **Implicit Euler**: A more stable integrator than explicit Euler, but requires simulating a linear system in each time step.
3. **Midpoint Method**: Similar to Euler's method, but uses the average of the current velocity and the velocity at the midpoint of the time step.
4. **Runge-Kutta 4th Order**: The most accurate integrator among the four, but also the most computationally expensive.


## Implementation

## Results and Discussion

### 1. The difference between integrators

### 2. Effect of parameters

1. **springCoef**:

2. **damperCoef**:

## Bonus

## Conclusion
