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
        self.e_psi_min = -math.pi / 2
        self.e_psi_max = math.pi / 2

        self.wheelbase = wheelbase

        self.state_reference = np.array([0, 0])

        self.optimization_problem = osqp.OSQP()

    def setup_optimization_problem(self, spatial_bicycle_model):
        A_linearized, B_linearized = spatial_bicycle_model.calculate_linearized_matrices()

        state = spatial_bicycle_model.calculate_spatial_state()

        nx, nu = B_linearized.shape
        lower_bound_equality, upper_bound_equality = self._make_state_dynamics_constraints(
            state, nx)

        lower_bound_inequality, upper_bound_inequality = self._make_state_and_input_constraints()

        # OSQP setup
        P, q = self._make_quadratic_and_linear_objective(nu)
        A_constraints = self._make_A_constraints(A_linearized, B_linearized)
        lower_bound = np.hstack([lower_bound_equality, lower_bound_inequality])
        upper_bound = np.hstack([upper_bound_equality, upper_bound_inequality])

        self.optimization_problem.setup(
            P=P, q=q, A=A_constraints, l=lower_bound, u=upper_bound,
            warm_start=True)

        # Keep track of lower_bound and upper_bound
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def compute_steering_angle(self, spatial_bicycle_model):
        A_linearized, B_linearized = spatial_bicycle_model.calculate_linearized_matrices()

        reference_curvature = spatial_bicycle_model.get_path_curvature()

        state = spatial_bicycle_model.calculate_spatial_state()
        print('State: e_y={}, e_psi={}'.format(state[0], state[1]))

        optimization_result = self._compute_optimal_control(
            A_linearized, B_linearized, state
        )

        _, nu = B_linearized.shape
        kappa_tilde = optimization_result.x[-self.prediction_horizon * nu: - (
            self.prediction_horizon - 1) * nu]

        if kappa_tilde[0] is None:
            print('Problem infeasible...')
            return None, None

        curvature = kappa_tilde[0] + reference_curvature

        predicted_poses = spatial_bicycle_model.calculate_predicted_poses(
            state, self.prediction_horizon)

        return self._convert_curvature_to_steering_angle(curvature), predicted_poses

    def _compute_optimal_control(self, A_linearized, B_linearized, state):
        nx, _ = B_linearized.shape
        A_constraints = self._make_A_constraints(A_linearized, B_linearized)

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

    def _make_A_constraints(self, A_linearized, B_linearized):
        nx, nu = B_linearized.shape

        Ax = sparse.kron(sparse.eye(self.prediction_horizon + 1), -sparse.eye(nx)) + \
            sparse.kron(sparse.eye(self.prediction_horizon + 1, k=-1), A_linearized)
        Bu = sparse.kron(sparse.vstack(
            [sparse.csc_matrix((1, self.prediction_horizon)),
             sparse.eye(self.prediction_horizon)]), B_linearized)

        A_equality = sparse.hstack([Ax, Bu])
        A_inequality = sparse.eye((self.prediction_horizon + 1)
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
            -self.Qn.dot(self.state_reference), np.zeros(self.prediction_horizon * nu)])

        return P, q

    def _convert_curvature_to_steering_angle(self, curvature):
        return np.arctan2(curvature * self.wheelbase, 1)
