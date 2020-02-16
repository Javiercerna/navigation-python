from scipy import sparse
import numpy as np
import osqp

import math


class MPC(object):
    def __init__(
            self, Q, R, Qn, prediction_horizon, kappa_tilde_min,
            kappa_tilde_max, wheelbase):
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

        self.wheelbase = wheelbase

        self.state_reference = np.array([0, 0])

        self.optimization_problem = osqp.OSQP()

    def setup_optimization_problem(
            self, spatial_bicycle_model, vehicle_pose, path):
        A, B, _ = spatial_bicycle_model.get_linearized_matrices(
            vehicle_pose, path.as_array(), path.path_curvature)

        state = spatial_bicycle_model.get_state(vehicle_pose, path.as_array())

        nx, nu = B.shape
        lower_bound_equality, upper_bound_equality = self._make_state_dynamics_constraints(
            state, nx)

        lower_bound_inequality, upper_bound_inequality = self._make_state_and_input_constraints()

        # OSQP setup
        P, q = self._make_quadratic_and_linear_objective(nu)
        A_constraints = self._make_A_constraints(A, B)
        lower_bound = np.hstack([lower_bound_equality, lower_bound_inequality])
        upper_bound = np.hstack([upper_bound_equality, upper_bound_inequality])

        self.optimization_problem.setup(
            P=P, q=q, A=A_constraints, l=lower_bound, u=upper_bound,
            warm_start=True)

        # Keep track of lower_bound and upper_bound
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def compute_steering_angle(
            self, spatial_bicycle_model, vehicle_pose, path):
        A, B, reference_curvature = spatial_bicycle_model.get_linearized_matrices(
            vehicle_pose, path.as_array(), path.path_curvature)

        state = spatial_bicycle_model.get_state(vehicle_pose, path.as_array())
        print('State: e_y={}, e_psi={}'.format(state[0], state[1]))

        optimization_result = self._compute_optimal_control(A, B, state)

        _, nu = B.shape
        kappa_tilde = optimization_result.x[-self.prediction_horizon * nu: - (
            self.prediction_horizon - 1) * nu]

        if kappa_tilde[0] is None:
            print('Problem infeasible...')
            return None

        curvature = kappa_tilde[0] + reference_curvature

        return self._convert_curvature_to_steering_angle(curvature)

    def _compute_optimal_control(self, A, B, state):
        nx, _ = B.shape
        A_constraints = self._make_A_constraints(A, B)

        self.lower_bound[:nx] = -state
        self.upper_bound[:nx] = -state
        self.optimization_problem.update(
            Ax=A_constraints.data, l=self.lower_bound, u=self.upper_bound)

        result = self.optimization_problem.solve()

        return result

    def _make_state_dynamics_constraints(self, state, nx):
        lower_bound_equality = np.hstack(
            [-state, np.zeros(self.prediction_horizon * nx)])
        upper_bound_equality = lower_bound_equality

        return lower_bound_equality, upper_bound_equality

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

    def _make_A_constraints(self, A, B):
        nx, nu = B.shape

        Ax = sparse.kron(sparse.eye(self.prediction_horizon + 1), -sparse.eye(nx)) + \
            sparse.kron(sparse.eye(self.prediction_horizon + 1, k=-1), A)
        Bu = sparse.kron(sparse.vstack(
            [sparse.csc_matrix((1, self.prediction_horizon)),
             sparse.eye(self.prediction_horizon)]), B)

        A_equality = sparse.hstack([Ax, Bu])
        A_inequality = sparse.eye((self.prediction_horizon+1)
                                  * nx + self.prediction_horizon * nu)

        A_constraints = sparse.vstack([A_equality, A_inequality], format='csc')

        return A_constraints

    def _make_quadratic_and_linear_objective(self, nu):
        P = sparse.block_diag([
            sparse.kron(sparse.eye(self.prediction_horizon), self.Q),
            self.Qn, sparse.kron(sparse.eye(self.prediction_horizon), self.R)],
            format='csc')

        q = np.hstack([
            np.kron(np.ones(self.prediction_horizon), -self.Q.dot(self.state_reference)),
            -self.Qn.dot(self.state_reference), np.zeros(self.prediction_horizon*nu)])

        return P, q

    def _convert_curvature_to_steering_angle(self, curvature):
        return np.arctan2(curvature * self.wheelbase, 1)
