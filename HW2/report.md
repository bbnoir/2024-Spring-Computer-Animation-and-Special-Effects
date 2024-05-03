# HW2 REPORT

110511010

## 1. Introduction

In this homework, we implemented the forward kinematics (FK) and inverse kinematics (IK). Given a skeleton and the motion data, we calculated the global coordinates of the joints in the skeleton using forward kinematics. For the inverse kinematics, we implemented the inverse Jacobian method to calculate the joint angles of the skeleton given the target position of the end effectors. If the target position is not reachable, the skeleton will move back to the previous position.  Also, I modified the step and epsilon values to see how they affect the results of IK.

## 2. Fundamentals

### 2.1 Structure of the Skeleton

The skeleton is a tree structure of bones. Each bone has its start position, end position, and local coordinate system. To render the skeleton, we need to get the global coordinates of the bones.

### 2.2 Local and Global Coordinates

Each bone has its local coordinate system. In the motion data, the rotation angles are given in the local coordinate system of the bones. So the bones rotate around their local axes based on the motion data. The global coordinate is the world coordinate system. To render the skeleton, we need to calculate the global coordinates of the bones, namely, to know the position and orientation of the bones in the world.

### 2.3 Forward Kinematics

The forward kinematics is to calculate the global coordinates of the bones given the rotation angles of the bones. We first calculate the global coordinates of the root bone, then calculate the global coordinates of the child bones based on the global coordinates of the parent bones. Using the notation in the slides, the global coordinates of bone `i` can be calculated as follows:

$$
_{i}T=^{i-1}_{0}RV_{i-1} + _{i-1}T
$$

Where $^{i-1}_{0}R$ is the rotation matrix that transform local coordinates of bone `i-1` to the global coordinate. $V_{i-1}$ is the local vector of bone `i`. These are calculated as follows:

$$
^{i-1}_{0}R = ^{1}_{0}R \cdot ^{2}_{1}R \cdots ^{i-1}_{i-2}R
$$

$$
V_{i-1} = \hat{V}_{i-1} l_{i-1}
$$

### 2.4 Inverse Kinematics

The inverse kinematics is to calculate the rotation angles of the bones given the target position of the end effectors. We use the inverse Jacobian method to calculate the rotation angles. The Jacobian matrix is in the form of:

$$
J = \begin{bmatrix}
\frac{\partial p}{\partial \theta_1} & \frac{\partial p}{\partial \theta_2} & \cdots & \frac{\partial p}{\partial \theta_n} \\
\end{bmatrix}
$$

Each column of the Jacobian matrix is the partial derivative of the position vector `p` with respect to the rotation angle `theta`. These columns are calculated as follows:

$$
\frac{\partial p}{\partial \theta_i} = a_i \times (p - r_i)
$$

Where `a_i` is the axis of rotation of bone `i`, `p` is the target position of the end effector, and `r_i` is the start position of the bone `i`.

In the inverse Jacobian method, we ratively update the rotation angles to minimize the error between the target position and the current position of the end effector. The update is as follows:

$$
\theta_{i+1} = \theta_i + \Delta t J^{-1}(\theta_i) V
$$

Where $\Delta t$ is the step size, `V` is the desired vector between the target position and the current position of the end effector.

## 3. Implementation

### 3.1 Forward Kinematics

### 3.2 Inverse Kinematics

### 3.3 Bonus 1: meet the limits of rotation angles

## 4. Results and Discussion

- IK: How different step and epsilon values affect the results?

## 5. Bonus

## 6. Conclusion

## 7. Demo Link
