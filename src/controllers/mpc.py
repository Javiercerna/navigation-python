from scipy import sparse
import numpy as np
import osqp

from models.spatial_bicycle_model import SpatialBicycleModel
from paths.path import Path

import math


class MPC(object):
    def __init__(
            self, Q, R, Qn, prediction_horizon, kappa_tilde_min,
            kappa_tilde_max):
        self.Q = Q
        self.R = R
        self.Qn = Qn
        self.prediction_horizon = prediction_horizon

        self.kappa_tilde_min = kappa_tilde_min
        self.kappa_tilde_max = kappa_tilde_max
        self.e_y_min = -3
        self.e_y_max = 3
        self.e_psi_min = -math.pi/2
        self.e_psi_max = math.pi/2

        self.state_reference = np.array([0, 0])

        self.optimization_problem = osqp.OSQP()

    def compute_steering_angle(self, A, B, state):
        A_equality, lower_bound_equality, upper_bound_equality = self._make_state_evolution_constraints(
            A, B, state)

        lower_bound_inequality, upper_bound_inequality = self._make_state_and_input_constraints()

        # OSQP setup
        P, q = self._make_quadratic_and_linear_objective(B)
        nx, nu = B.shape
        Aineq = sparse.eye((self.prediction_horizon+1)
                           * nx + self.prediction_horizon * nu)
        A = sparse.vstack([A_equality, Aineq], format='csc')
        lower_bound = np.hstack([lower_bound_equality, lower_bound_inequality])
        upper_bound = np.hstack([upper_bound_equality, upper_bound_inequality])

        try:
            self.optimization_problem.setup(
                P=P, q=q, A=A, l=lower_bound, u=upper_bound, warm_start=True)
        except:
            lower_bound[:nx] = -state
            upper_bound[:nx] = -state
            self.optimization_problem.update(l=lower_bound, u=upper_bound)

        result = self.optimization_problem.solve()

        return result

    def _make_quadratic_and_linear_objective(self, B):
        _, nu = B.shape

        P = sparse.block_diag([
            sparse.kron(sparse.eye(self.prediction_horizon), self.Q),
            self.Qn, sparse.kron(sparse.eye(self.prediction_horizon), self.R)],
            format='csc')

        q = np.hstack([
            np.kron(np.ones(self.prediction_horizon), -self.Q.dot(self.state_reference)),
            -self.Qn.dot(self.state_reference), np.zeros(self.prediction_horizon*nu)])

        return P, q

    def _make_state_evolution_constraints(self, A, B, state):
        nx, _ = B.shape

        Ax = sparse.kron(sparse.eye(self.prediction_horizon + 1), -sparse.eye(nx)) + \
            sparse.kron(sparse.eye(self.prediction_horizon + 1, k=-1), A)
        Bu = sparse.kron(sparse.vstack(
            [sparse.csc_matrix((1, self.prediction_horizon)),
             sparse.eye(self.prediction_horizon)]), B)

        A_equality = sparse.hstack([Ax, Bu])
        lower_bound_equality = np.hstack(
            [-state, np.zeros(self.prediction_horizon * nx)])
        upper_bound_equality = lower_bound_equality

        return A_equality, lower_bound_equality, upper_bound_equality

    def _make_state_and_input_constraints(self):
        state_min = np.array([self.e_y_min, self.e_psi_min])
        state_max = np.array([self.e_y_max, self.e_psi_max])

        input_min = np.array([self.kappa_tilde_min])
        input_max = np.array([self.kappa_tilde_max])

        lower_bound_state = np.kron(
            np.ones(self.prediction_horizon + 1), state_min)
        lower_bound_input = np.kron(
            np.ones(self.prediction_horizon), input_min)
        lower_bound = np.hstack([lower_bound_state, lower_bound_input])

        upper_bound_state = np.kron(
            np.ones(self.prediction_horizon + 1), state_max)
        upper_bound_input = np.kron(
            np.ones(self.prediction_horizon), input_max)
        upper_bound = np.hstack([upper_bound_state, upper_bound_input])

        return lower_bound, upper_bound
