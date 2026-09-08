# Robotics Kinematics & Motion Studio 🤖🦾

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Domain](https://img.shields.io/badge/Domain-Robotics%20%26%20Mechatronics-purple.svg)](#)
[![Tech](https://img.shields.io/badge/Tech-Denavit--Hartenberg%20%7C%20Inverse%20Kinematics%20%7C%20Python-brightgreen.svg)](#)

An interactive robotics motion planning and kinematics simulator implementing **Denavit-Hartenberg (D-H) 4x4 Homogeneous Transformation Matrices**, **Analytical & Numerical Inverse Kinematics (IK)**, and **Manipulator Jacobian Singularity Analysis**.

---

## 🌟 Key Engineering Features

- 🦾 **Forward Kinematics (FK):** Computes precise end-effector spatial coordinates from joint angle vectors using homogeneous D-H transformation matrices:
  $$T_0^n = A_1 A_2 \cdots A_n$$
- 🎯 **Inverse Kinematics (IK):** Analytical trigonometric and algebraic solvers mapping Cartesian targets $(X, Y, Z)$ back into joint actuator space.
- 📐 **Workspace & Singularity Boundary Checks:** Real-time geometric limits preventing joint saturation and Jacobian singularities ($\det(J) \to 0$).
- 🖥️ **Interactive Manipulator Canvas:** Live interactive dragging and kinematic joint manipulation.
- 🐍 **Python Robotics Engine:** Includes standalone Python script (`kinematics_solver.py`) for analytical D-H matrix chaining.

---

## 🚀 Quick Start

### 1. Run Interactive Kinematics Studio:
```bash
git clone https://github.com/rushikeshgarad2024-dev/robotics-kinematics-studio.git
cd robotics-kinematics-studio
npx serve .
```

### 2. Run Python Kinematics Solver:
```bash
python kinematics_solver.py
```

---

## 👨‍💻 Author
**Rushikesh Garad** - [GitHub](https://github.com/rushikeshgarad2024-dev) • [Email](mailto:rushikeshgarad2024@gmail.com)
