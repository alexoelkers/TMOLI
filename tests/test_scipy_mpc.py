#Required to find the tests
import sys
sys.path.append("~/TMOLI/")

import unittest
import numpy as np
from examples.scipy_mpc import Scipy_MPC

class TestSciPyMPC(unittest.TestCase):
    def test_assert_true(self):
        """
        Checks that the testing itself works. If this doesn't pass, we have an issue!
        """
        self.assertTrue(True)

    def test_creation_empty(self):
        mpc = Scipy_MPC()
        self.assertEqual(type(mpc), Scipy_MPC)

    def test_creation_final(self):
        x_final = np.array([1, 2, 3])
        mpc = Scipy_MPC(x_final)
        np.testing.assert_allclose(x_final, mpc.get_target_state())

    def test_get_steps(self):
        mpc = Scipy_MPC()
        self.assertEqual(mpc.get_steps(), 20)
    

class TestOptimizer(unittest.TestCase):
    def test_states_one_row(self):
        mpc = Scipy_MPC()
        variables = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        x = np.array([
            [1, 6],
            [2, 7], 
            [3, 8]
        ])
        u = np.array([
            [4, 9],
            [5, 10]
        ])
        x_given, u_given = mpc._get_states(variables)

        np.testing.assert_allclose(x_given, x)
        np.testing.assert_allclose(u_given, u)

    def test_dim_mismatch(self):
        mpc = Scipy_MPC()
        variables = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9])
        with self.assertRaises(ValueError):
            mpc._get_states(variables)
        with self.assertRaises(ValueError):
            mpc.cost_fun(variables)
    
    def test_cost(self):
        mpc = Scipy_MPC()
        variables = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        cost = mpc.cost_fun(variables)
        print(cost)
        np.testing.assert_allclose(cost, 192.5)

    def test_optimizer(self):
        goal = np.array([2, 2, 0])
        mpc = Scipy_MPC(goal)
        success, u, msg = mpc.optimize()
        self.assertTrue(success)
