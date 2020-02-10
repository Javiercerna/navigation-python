from scipy import sparse
import numpy as np
import osqp

from spatial_bicycle_model import SpatialBicycleModel
from path import Path

import math


class MPC(object):
    def __init__(
            self, Q, R, Qn, prediction_horizon, steering_angle_min,
            steering_angle_max):
        self.Q = Q
        self.R = R
        self.Qn = Qn
        self.prediction_horizon = prediction_horizon

        self.steering_angle_min = steering_angle_min
        self.steering_angle_max = steering_angle_max
        self.e_y_min = -np.inf
        self.e_y_max = np.inf
        self.e_phi_min = -math.pi
        self.e_phi_max = math.pi

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
            pass

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
        state_min = np.array([self.e_y_min, self.e_phi_min])
        state_max = np.array([self.e_y_max, self.e_phi_max])

        input_min = np.array([self.steering_angle_min])
        input_max = np.array([self.steering_angle_max])

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


if __name__ == '__main__':
    # P = sparse.csc_matrix([[4, 1], [1, 2]])
    # q = np.array([1, 1])
    # A = sparse.csc_matrix([[1, 1], [1, 0], [0, 1]])
    # l = np.array([1, 0, 0])
    # u = np.array([1, 0.7, 0.7])

    # prob = osqp.OSQP()

    # prob.setup(P=P, q=q, A=A, l=l, u=u)

    # res = prob.solve()

    # print(res.x)

    Q = sparse.diags([1., 1.])
    Qn = Q
    R = 0.1*sparse.eye(1)
    prediction_horizon = 3

    mpc = MPC(Q=Q, R=R, Qn=Qn, prediction_horizon=prediction_horizon,
              steering_angle_min=-math.pi/3, steering_angle_max=math.pi/3)

    v = 1
    dt = 0.1

    vehicle_pose = np.array([3, 4, math.atan(4) + 2*math.pi])

    waypoints_x = [x for x in range(0, 10)]
    waypoints_y = [x for x in range(0, 10)]

    path = Path(waypoints_x=waypoints_x, waypoints_y=waypoints_y)
    path_curvatures = path.path_curvature

    spatial_bicycle_model = SpatialBicycleModel(ds=v*dt)

    A, B, _ = spatial_bicycle_model.get_linearized_matrices(
        vehicle_pose, path.as_array(), path_curvatures)

    curvature_reference = 0

    state = spatial_bicycle_model.get_state(vehicle_pose, path.as_array())
    result = mpc.compute_steering_angle(A, B, state)

    _, nu = B.shape
    k_tilde = result.x[-prediction_horizon*nu:-(prediction_horizon-1)*nu]
    curvature = k_tilde + curvature_reference

    print(curvature)
