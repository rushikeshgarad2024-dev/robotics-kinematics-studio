"""
Robotics 3-DOF / 6-DOF Forward & Inverse Kinematics Solver
Author: Rushikesh Garad
"""
import numpy as np

def dh_transform(theta, d, a, alpha):
    """
    Computes standard Denavit-Hartenberg transformation matrix A_i.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1]
    ])

def forward_kinematics(joint_angles_rad, link_lengths=[1.0, 1.0, 0.5]):
    """
    3-DOF Planar/Spatial Manipulator Forward Kinematics.
    """
    t1, t2, t3 = joint_angles_rad
    l1, l2, l3 = link_lengths

    A1 = dh_transform(t1, 0, l1, 0)
    A2 = dh_transform(t2, 0, l2, 0)
    A3 = dh_transform(t3, 0, l3, 0)

    T_0_3 = A1 @ A2 @ A3
    end_effector_pos = T_0_3[:3, 3]
    return end_effector_pos

def inverse_kinematics_2d(target_x, target_y, l1=1.0, l2=1.0):
    """
    Analytical 2-DOF Inverse Kinematics.
    """
    r_sq = target_x**2 + target_y**2
    cos_t2 = (r_sq - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(cos_t2) > 1.0:
        raise ValueError("Target position is outside robot reachable workspace!")

    sin_t2 = np.sqrt(1 - cos_t2**2) # Elbow up
    t2 = np.arctan2(sin_t2, cos_t2)
    k1 = l1 + l2 * cos_t2
    k2 = l2 * sin_t2
    t1 = np.arctan2(target_y, target_x) - np.arctan2(k2, k1)

    return np.degrees(t1), np.degrees(t2)

if __name__ == "__main__":
    x_target, y_target = 1.2, 0.8
    t1, t2 = inverse_kinematics_2d(x_target, y_target)
    print(f"Target Point: ({x_target}, {y_target})")
    print(f"Computed Joint Angles -> Theta 1: {t1:.2f}°, Theta 2: {t2:.2f}°")
