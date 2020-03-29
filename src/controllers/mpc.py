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
        self.nx = 2
        self.nu = 1

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
        state = spatial_bicycle_model.calculate_spatial_state()

        lower_bound_equality, upper_bound_equality = self._make_state_dynamics_constraints(
            state)

        lower_bound_inequality, upper_bound_inequality = self._make_state_and_input_constraints()

        # OSQP setup
        P, q = self._make_quadratic_and_linear_objective()
        A_constraints = self._make_A_constraints(spatial_bicycle_model)
        lower_bound = np.hstack([lower_bound_equality, lower_bound_inequality])
        upper_bound = np.hstack([upper_bound_equality, upper_bound_inequality])

        self.optimization_problem.setup(
            P=P, q=q, A=A_constraints, l=lower_bound, u=upper_bound,
            warm_start=True)

        # Keep track of lower_bound and upper_bound
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def compute_steering_angle(self, spatial_bicycle_model):
        reference_curvature = spatial_bicycle_model.get_path_curvature()

        state = spatial_bicycle_model.calculate_spatial_state()
        print('State: e_y={}, e_psi={}'.format(state[0], state[1]))

        optimization_result = self._compute_optimal_control(
            spatial_bicycle_model)

        kappa_tilde = optimization_result.x[-self.prediction_horizon * self.nu: - (
            self.prediction_horizon - 1) * self.nu]

        if kappa_tilde[0] is None:
            print('Problem infeasible...')
            return None, None

        curvature = kappa_tilde[0] + reference_curvature

        predicted_poses = spatial_bicycle_model.calculate_predicted_poses(
            state, self.prediction_horizon)

        return self._convert_curvature_to_steering_angle(curvature), predicted_poses

    def _compute_optimal_control(self, spatial_bicycle_model):
        state = spatial_bicycle_model.calculate_spatial_state()

        A_constraints = self._make_A_constraints(spatial_bicycle_model)

        self.lower_bound[0:self.nx] = -state
        self.upper_bound[0:self.nx] = -state

        self.optimization_problem.update(
            Ax=A_constraints.data, l=self.lower_bound, u=self.upper_bound)

        result = self.optimization_problem.solve()

        return result

    def _make_state_dynamics_constraints(self, state):
        lower_bound_equality = np.hstack(
            [-state, np.zeros(self.prediction_horizon * self.nx)])
        upper_bound_equality = lower_bound_equality

        return lower_bound_equality, upper_bound_equality

    def _make_state_and_input_constraints(self):
        state_min = np.array([self.e_y_min, self.e_psi_min])
        state_max = np.array([self.e_y_max, self.e_psi_max])

        input_min = np.array([self.kappa_tilde_min])
        input_max = np.array([self.kappa_tilde_max])

        lower_bound = self._make_inequality_constraint(state_min, input_min)
        upper_bound = self._make_inequality_constraint(state_max, input_max)

        return lower_bound, upper_bound

    def _make_inequality_constraint(self, bounds_state, bounds_input):
        # Inequality constraint is always a vector (1 x k)
        inequality_constraint = np.array([])

        # Constraints for state(0), state(1), ..., state(N)
        for _ in range(self.prediction_horizon + 1):
            inequality_constraint = np.hstack(
                [inequality_constraint, bounds_state]
            )

        # Constraints for input(0), input(1), ..., input(N-1)
        for _ in range(self.prediction_horizon):
            inequality_constraint = np.hstack(
                [inequality_constraint, bounds_input]
            )

        return inequality_constraint

    def _make_A_constraints(self, spatial_bicycle_model):
        Ax = -sparse.eye(self.nx * (self.prediction_horizon + 1))
        Ax = Ax.tocsr()

        Bu = sparse.csc_matrix((self.nx * (self.prediction_horizon + 1),
                                self.nu * (self.prediction_horizon)))

        for horizon_step in range(self.prediction_horizon):
            A_linearized, B_linearized = spatial_bicycle_model.calculate_linearized_matrices(
                horizon_step)

            Ax[(horizon_step + 1) * self.nx: (horizon_step + 2) * self.nx,
               horizon_step * self.nx: (horizon_step + 1) * self.nx] = A_linearized
            Bu[(horizon_step + 1) * self.nx: (horizon_step + 2) * self.nx,
               horizon_step * self.nu: (horizon_step + 1) * self.nu] = B_linearized

        A_equality = sparse.hstack([Ax, Bu])

        A_inequality = sparse.eye(
            (self.prediction_horizon + 1) * self.nx +
            self.prediction_horizon * self.nu
        )

        A_constraints = sparse.vstack([A_equality, A_inequality], format='csc')

        return A_constraints

    def _make_quadratic_and_linear_objective(self):
        P = sparse.block_diag(
            [
                sparse.kron(sparse.eye(self.prediction_horizon), self.Q),
                self.Qn,
                sparse.kron(sparse.eye(self.prediction_horizon), self.R)
            ],
            format='csc'
        )

        q = np.hstack(
            [
                np.kron(np.ones(self.prediction_horizon), -self.Q.dot(self.state_reference)),
                -self.Qn.dot(self.state_reference),
                np.zeros(self.prediction_horizon * self.nu)
            ]
        )

        return P, q

    def _convert_curvature_to_steering_angle(self, curvature):
        return np.arctan2(curvature * self.wheelbase, 1)
