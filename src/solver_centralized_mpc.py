"""
Solver for a centralized MPC problem. All vehicle problems are solved at once. Follower vehicles
uses state trajectories from preceding vehicle.
"""

import numpy
import cvxpy
import scipy.sparse as sparse


class MPC(object):

    def __init__(self, vehicle_amount, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None):

        v_slack_cost_factor = 100
        pos_slack_cost_factor = 10000
        safety_slack_cost_factor = 10000

        self.nx = Ad.shape[0]
        self.nu = Bd.shape[1]

        self.inf = 1000000  # "Infinity" used when there are no limits given.

        if xmin is None:
            xmin = -self.inf*numpy.ones(self.nx)  # No min state limit.

        if xmax is None:
            xmax = self.inf*numpy.ones(self.nx)  # No max state limit.

        if umin is None:
            umin = -self.inf*numpy.ones(self.nu)  # No min input limit.

        if umax is None:
            umax = self.inf*numpy.ones(self.nu)  # No max input limit.

        self.n = vehicle_amount
        self.dt = delta_t
        self.h = horizon
        self.zeta = zeta
        self.Q = Q

        self.iterations = 0
        self.solver_iterations = 0

        # Create problem.
        self.prob = cvxpy.Problem(cvxpy.Minimize(1))

        # Optimization variables.
        self.x = cvxpy.Variable((self.h + 1) * self.nx * self.n)
        self.u = cvxpy.Variable(self.h * self.nu * self.n)
        self.v_slack = cvxpy.Variable((self.h + 1) * self.n)
        self.pos_slack = cvxpy.Variable((self.h + 1) * self.n)
        self.safety_slack = cvxpy.Variable((self.h + 1) * (self.n - 1))

        # Optimization constraints.
        x0_constraints = self._get_x0_constraints(numpy.zeros(self.nx * self.n))
        state_constraints_lower = self._get_lower_state_constraints(xmin)
        state_constraints_upper = self._get_upper_state_constraints(xmax)
        input_constraints_upper, input_constraints_lower = self._get_input_constraints(umin, umax)
        dynamics_constraints = self._get_dynamics_constraints(Ad, Bd)
        safety_constraints = self._get_safety_constraints(truck_length, safety_distance)
        v_slack_constraints = self._get_v_slack_constraints()
        pos_slack_constraints = self._get_pos_slack_constraints()
        safety_slack_constraints = self._get_safety_slack_constraints()

        self.prob.constraints = x0_constraints              # Constraint updated each iteration.
        self.prob.constraints += state_constraints_lower
        self.prob.constraints += state_constraints_upper
        self.prob.constraints += input_constraints_lower
        self.prob.constraints += input_constraints_upper
        self.prob.constraints += dynamics_constraints
        self.prob.constraints += v_slack_constraints
        self.prob.constraints += pos_slack_constraints

        if self.n > 1:
            self.prob.constraints += safety_constraints
            self.prob.constraints += safety_slack_constraints

        # Cost functions.
        self.v_slack_cost = self._get_v_slack_cost(v_slack_cost_factor)
        self.pos_slack_cost = self._get_pos_slack_cost(pos_slack_cost_factor)
        self.safety_slack_cost = self._get_safety_slack_cost(safety_slack_cost_factor)
        self.timegap_cost = self._get_timegap_cost(timegap)

        # Pre-computed matrices for reference tracking cost.
        self.state_P = sparse.kron(sparse.eye((self.h + 1) * self.n), (1 - zeta)*Q)
        self.state_Q_diag = sparse.kron(sparse.eye((self.h + 1) * self.n), -(1 - zeta)*Q)
        self.input_P = sparse.kron(sparse.eye(self.h * self.n), R)
        self.R_diag = sparse.kron(sparse.eye(self.h * self.n), -R)

    def solve_mpc(self, speed_profile, x0s):
        """
        Solves the MPC problem.
        :param speed_profile: speed profile, instance of speed_profile.Speed.
        :param x0s: array containing the initial values [velocity(0)_0, position(0)_0,
        velocity(0)_1, position(0)_1, ...] for all vehicles 0, ..., n-1.
        :return: None
        """
        """Solves the MPC problem. """
        self._update_x0_constraints(x0s)

        xref, uref = self._get_references(speed_profile, x0s)

        cost = self._get_cost(xref, uref)

        self.prob.objective = cvxpy.Minimize(cost)

        try:
            self.prob.solve(solver='ECOS')
            self.iterations += 1
            self.solver_iterations += self.prob.solver_stats.num_iters
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(self.prob.status))

    def _update_x0_constraints(self, x0s):
        """Updates the constraint of the problem corresponding to x0. """
        self.prob.constraints[0] = self._get_x0_constraints(x0s)[0]

    def _get_references(self, speed_profile, x0s):
        """Returns the state and input references obtained from current positions and optimal
        speed profile. """
        pos_ref = numpy.zeros((self.h + 1) * self.n)

        uref = numpy.zeros(self.h * self.nu * self.n)

        # Position reference is obtained from current position and speed profile.
        for j in range(self.n):
            pos_ref[(self.h + 1) * j] = x0s[j*2 + 1]
            for i in range((self.h + 1) * j + 1, (self.h + 1) * (j + 1)):
                pos_ref[i] = pos_ref[i - 1] + self.dt * speed_profile.get_speed_at(pos_ref[i - 1])

        # Velocity reference is obtained from the speed profile at the reference positions.
        vel_ref = speed_profile.get_speed_at(pos_ref)

        # Acceleration reference is obtained from the velocity reference.
        for j in range(self.n):
            uref[self.h * j:self.h * (j + 1)] = \
                (vel_ref[(self.h + 1) * j + 1:(self.h + 1) * (j + 1) + 0] -
                 vel_ref[(self.h + 1) * j + 0:(self.h + 1) * (j + 1) - 1]) / self.dt

        # State reference consists of velocities and positions.
        xref = self._interleave_vectors(vel_ref, pos_ref)

        return xref, uref

    def _get_cost(self, xref, uref):
        """Returns the cost for the MPC problem. Uses pre-computed matrices and costs in order to
        speed up computation. Called each iteration. """

        state_reference_cost = 0.5 * cvxpy.quad_form(self.x, self.state_P) + \
                               self.state_Q_diag.dot(xref) * self.x

        input_reference_cost = 0.5 * cvxpy.quad_form(self.u, self.input_P) + \
                               self.R_diag.dot(uref) * self.u

        cost = self.v_slack_cost + self.pos_slack_cost + state_reference_cost + input_reference_cost

        if self.n > 1:
            cost += self.safety_slack_cost

            if numpy.mean(xref[0::2]) > 0.1:
                cost += self.timegap_cost

        return cost

    def _get_lower_state_constraints(self, xmin):
        """Returns the lower constraints for the states. Called on initialization. """
        AX = sparse.eye((self.h + 1) * self.nx * self.n)
        lower = numpy.tile(xmin, (self.h + 1) * self.n)

        constraint = [AX * self.x >= lower]

        return constraint

    def _get_upper_state_constraints(self, xmax):
        """Returns the upper constraints for the states. Includes the slack variable for velocity
        limitation. Called on initialization. """
        AX = sparse.kron(sparse.eye((self.h + 1) * self.n), [1, 0])
        upper = numpy.ones((self.h + 1) * self.n)*xmax[0]

        constraint = [AX*self.x - self.v_slack <= upper]

        return constraint

    def _get_input_constraints(self, umin, umax):
        """Returns the constraints corrseponding to input limits. Called on initialization. """
        AU = numpy.eye(self.h * self.nu * self.n)
        lower = numpy.tile(umin, self.h * self.n)
        upper = numpy.tile(umax, self.h * self.n)

        constraint = [AU * self.u <= upper], [AU * self.u >= lower]

        return constraint

    def _get_dynamics_constraints(self, Ad, Bd):
        """Returns the constraints for x(k+1) = Ax(k) + Bu(k). Called on initialization. """
        a = sparse.kron(sparse.hstack([sparse.eye(self.h), sparse.csc_matrix((self.h, 1))]), Ad) + \
            sparse.kron(sparse.hstack([sparse.csc_matrix((self.h, 1)), sparse.eye(self.h)]),
                        -sparse.eye(self.nx))

        AX = sparse.kron(sparse.eye(self.n), a)

        BB = sparse.kron(sparse.eye(self.h * self.n), Bd)

        constraint = [AX * self.x + BB * self.u == 0]

        return constraint

    def _get_x0_constraints(self, x0s):
        """Returns the constraints for x(0) = x0 for each vehicle. Called on each iteration. """
        a = sparse.lil_matrix((self.n, (self.h + 1) * self.n))
        for i in range(self.n):
            a[i, (self.h + 1)*i] = 1

        AX = sparse.kron(a, sparse.eye(2))

        constraint = [AX * self.x == x0s]

        return constraint

    def _get_safety_constraints(self, truck_length, safety_distance):
        """Returns the constraints for keeping safety distance for the follower vehicles. Called
        on initialization.
        Constraint: preceding position - follower position - truck length > safety distance. """
        if self.n <= 1:
            return []

        # Matrix for getting preceding position.
        matrix_preceding_position = sparse.kron(
            sparse.hstack([sparse.eye((self.h + 1)*(self.n - 1)),
                           sparse.csc_matrix(((self.h + 1)*(self.n - 1), self.h + 1))]),
            [0, 1])

        # Matrix for getting follower position
        matrix_follower_position = sparse.kron(
            sparse.hstack([sparse.csc_matrix(((self.h + 1)*(self.n - 1), self.h + 1)),
                           sparse.eye((self.h + 1)*(self.n - 1))]),
            [0, 1])

        AX = matrix_follower_position - matrix_preceding_position

        constraint = [AX * self.x - self.safety_slack < - truck_length - safety_distance]

        return constraint

    def _get_v_slack_constraints(self):
        """Returns the constraints that the velocity slack variable is positive. Called on
        initialization. """
        constraint = [self.v_slack > 0]

        return constraint

    def _get_pos_slack_constraints(self):
        """Returns the constraints that the position slack variable is positive. Called on
        initialization. """
        constraint = [self.pos_slack > 0]

        return constraint

    def _get_safety_slack_constraints(self):
        """Returns the constraints that the safety distance slack variable is positive. Called on
        initialization. """
        if self.n > 1:
            constraint = [self.safety_slack > 0]
        else:
            constraint = []

        return constraint

    def _get_v_slack_cost(self, cost_factor):
        """Returns the cost function for the velocity slack variable. The cost is quadratic.
        Called on initialization. """
        v_slack_P = numpy.eye((self.h + 1) * self.n) * cost_factor

        cost = cvxpy.quad_form(self.v_slack, v_slack_P)

        return cost

    def _get_pos_slack_cost(self, cost_factor):
        """Returns the cost function for the position slack variable. The cost is quadratic.
        Called on initialization. """
        pos_slack_P = numpy.eye((self.h + 1) * self.n) * cost_factor

        cost = cvxpy.quad_form(self.pos_slack, pos_slack_P)

        return cost

    def _get_safety_slack_cost(self, cost_factor):
        """Returns the cost function for the safety distance slack variable. The cost is quadratic.
        Called on initialization. """
        if self.n > 1:
            safety_slack_P = numpy.eye((self.h + 1) * (self.n - 1)) * cost_factor

            cost = cvxpy.quad_form(self.safety_slack, safety_slack_P)
        else:
            cost = []

        return cost

    def _get_timegap_cost(self, timegap):
        """Returns the cost function for timegap tracking. Called on initialization.
        The cost is a quadratic cost with P = zeta*Q. The cost is minimized when
        (preceding position - follower position) / (follower velocity) = timegap. """
        if self.n <= 1:
            return []

        # Matrix for getting preceding position.
        matrix_preceding_position = sparse.kron(
            sparse.hstack([sparse.eye((self.h + 1)*(self.n - 1)),
                           sparse.csc_matrix(((self.h + 1)*(self.n - 1), self.h + 1))]),
            [0, 1])

        # Matrix for getting follower position.
        matrix_follower_position = sparse.kron(
            sparse.hstack([sparse.csc_matrix(((self.h + 1)*(self.n - 1), self.h + 1)),
                           sparse.eye((self.h + 1)*(self.n - 1))]),
            [0, 1])

        # Matrix for the velocity of the follower vehicle.
        matrix_follower_velocity = sparse.kron(
            sparse.hstack([sparse.csc_matrix(((self.h + 1)*(self.n - 1), self.h + 1)),
                           sparse.eye((self.h + 1)*(self.n - 1))]),
            [-timegap, 0])

        # AX*x = preceding_position - follower position - timegap*(follower velocity)
        AX = matrix_preceding_position - matrix_follower_position + matrix_follower_velocity

        # Final cost is (AX*x)'P(AX*x).
        P = sparse.kron(sparse.eye((self.h + 1) * (self.n - 1)), self.zeta*self.Q[1, 1])

        timegap_cost = cvxpy.quad_form(AX*self.x, P)

        return timegap_cost

    def add_position_constraint(self, position):
        """Adds a constraint for maximum position. """
        AX = sparse.kron(sparse.eye((self.h + 1)*self.n), [0, 1])

        constraint = [AX*self.x - self.pos_slack <= position]

        self.prob.constraints += constraint

    def get_instantaneous_accelerations(self):
        """Returns an array of accelerations containing the first control input for each vehicle.
        :return: numpy array [acceleration0, acceleration1, ...] for all vehicles 0, ..., n-1.
        """
        try:
            u_opt = numpy.squeeze(numpy.array(self.u.value))
            accelerations = u_opt[0::self.h]
        except (TypeError, IndexError) as e:
            accelerations = numpy.zeros(self.n)
            print('MPC returning acc = 0: {}'.format(e))

        return accelerations

    @staticmethod
    def _interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c
