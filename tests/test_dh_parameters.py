import unittest
import numpy as np

class TestKinematics(unittest.TestCase):
    def test_homogenous_transformation_matrix(self):
        theta = np.pi / 2
        d = 0.5
        a = 1.0
        alpha = 0
        T = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),                np.cos(alpha),               d],
            [0,             0,                            0,                           1]
        ])
        self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1.0, places=5)

if __name__ == '__main__':
    unittest.main()
