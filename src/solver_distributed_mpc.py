"""
Solver for a MPC problem where the vehicle, if it is a follower, also uses trajectories from
the preceding vehicle in cost and constraints.
"""


import numpy
import cvxpy
import scipy.sparse as sparse


class MPC(object):

    def __init__(self, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length, safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, is_leader=False):

        self.dt = delta_t
        self.h = horizon
        self.truck_length = truck_length
        self.safety_distance = safety_distance
        self.timegap = timegap

        self.nx = Ad.shape[0]
        self.nu = Bd.shape[1]

        self.x = cvxpy.Variable((self.h + 1) * self.nx)
        self.u = cvxpy.Variable(self.h * self.nu)

        v_slack_cost_factor = 100
        self.v_slack = cvxpy.Variable(self.h + 1)

        pos_slack_cost_factor = 100
        self.pos_slack = cvxpy.Variable(self.h + 1)

        safety_slack_cost_factor = 100
        self.safety_slack = cvxpy.Variable(self.h + 1)

        self.inf = 1000000  # "Infinity" used when there are no limits given.

        self.is_leader = is_leader

        self.status = 'OK'  # Status of solver.

        if xmin is None:
            xmin = -self.inf*numpy.ones(self.nx)  # No min state limit.

        if xmax is None:
            xmax = self.inf*numpy.ones(self.nx)  # No max state limit.

        if umin is None:
            umin = -self.inf*numpy.ones(self.nu)  # No min input limit.

        if umax is None:
            umax = self.inf*numpy.ones(self.nu)  # No max input limit.

        if x0 is None:
            x0 = numpy.zeros(self.nx)  # Origin default initial condition.
        else:
            x0 = x0
        self.x0 = x0

        self.iterations = 0
        self.solver_iterations = 0

        # Problem.
        self.prob = cvxpy.Problem(cvxpy.Minimize(1))

        # Problem constraints.
        state_constraint_lower = self._get_lower_state_constraints(xmin)
        state_constraint_upper = self._get_upper_state_constraints(xmax)
        input_constraint_lower, input_constraint_upper = self._get_input_constraints(umin, umax)
        x0_constraints = self._get_x0_constraint(x0)       # Update during mpc.
        dynamics_constraints = self._get_dynamics_constraints(Ad, Bd)
        slack_constraint_v, slack_constraint_safety = self._get_slack_constraints()
        pos_slack_constraints = self._get_pos_slack_constraints()

        self.prob.constraints = state_constraint_lower
        self.prob.constraints += state_constraint_upper
        self.prob.constraints += input_constraint_lower
        self.prob.constraints += input_constraint_upper
        self.prob.constraints += x0_constraints         # Update during mpc.
        self.prob.constraints += dynamics_constraints
        self.prob.constraints += slack_constraint_v
        self.prob.constraints += slack_constraint_safety
        if not self.is_leader:
            # Update during mpc.
            self.prob.constraints += self._get_safety_constraints(0, [0], [0])
        self.prob.constraints += pos_slack_constraints

        # Pre-compute costs for slack variables.
        self.v_slack_cost = self._get_v_slack_cost(v_slack_cost_factor)
        self.pos_slack_cost = self._get_pos_slack_cost(pos_slack_cost_factor)
        self.safety_slack_cost = self._get_safety_slack_cost(safety_slack_cost_factor)

        # Pre-compute matrices used for cost calculation in order to speed up computation.
        self.state_P = sparse.kron(sparse.eye(self.h + 1), (1 - zeta)*Q)
        self.input_P = sparse.kron(sparse.eye(self.h), R)
        self.timegap_P = sparse.kron(sparse.eye(self.h + 1), zeta*Q)

        self.state_Q_diag = sparse.kron(sparse.eye(self.h + 1), -(1 - zeta)*Q)
        self.timegap_Q_diag = sparse.kron(sparse.eye(self.h + 1), -zeta * Q)
        self.R_diag = sparse.kron(sparse.eye(self.h), -R)

        # self.timegap_shift = int(round(self.timegap / self.dt))

    def _get_lower_state_constraints(self, xmin):
        """Returns the lower constraints for the states. Called on initialization. """
        AX = sparse.eye((self.h + 1) * self.nx)
        lower = numpy.tile(xmin, self.h + 1)

        return [AX * self.x >= lower]

    def _get_upper_state_constraints(self, xmax):
        """Returns the upper constraints for the states. Includes the slack variable for velocity
        limitation. Called on initialization. """
        AX = sparse.kron(sparse.eye(self.h + 1), [1, 0])
        upper = numpy.tile(xmax[0], self.h + 1)

        return [AX*self.x - self.v_slack <= upper]

    def _get_input_constraints(self, umin, umax):
        """Returns the constraints corrseponding to input limits. Called on initialization. """
        AU = numpy.eye(self.h*self.nu)
        lower = numpy.tile(umin, self.h)
        upper = numpy.tile(umax, self.h)

        return [AU * self.u >= lower], [AU * self.u <= upper]

    def _get_x0_constraint(self, x0):
        """Returns the constraints specifying that x(0) = x0. Called each iteration. """
        AA = sparse.hstack([sparse.eye(self.nx), sparse.csc_matrix((self.nx, self.h*self.nx))])

        constraint = [AA * self.x == x0]

        return constraint

    def _get_dynamics_constraints(self, Ad, Bd):
        """Returns the constraints for x(k+1) = Ax(k) + Bu(k). Called on initialization. """
        AA = sparse.kron(sparse.hstack([sparse.eye(self.h), sparse.csc_matrix((self.h, 1))]),
                         Ad) + \
             sparse.kron(sparse.hstack([sparse.csc_matrix((self.h, 1)), sparse.eye(self.h)]),
                         -sparse.eye(self.nx))

        BB = sparse.kron(sparse.eye(self.h), Bd)

        constraint = [AA * self.x + BB * self.u == 0]

        return constraint

    def _get_safety_constraints(self, current_time, preceding_timestamps, preceding_positions):
        """Returns the constraints for keeping a safety distance. Called each iteration. """
        target_time = current_time + numpy.arange(self.h + 1) * self.dt
        preceding_pos = numpy.interp(target_time, preceding_timestamps, preceding_positions)

        AX = sparse.kron(sparse.eye(self.h + 1), [0, 1])

        posmax = preceding_pos - self.truck_length - self.safety_distance

        constraints = [AX * self.x - self.safety_slack < posmax]

        return constraints

    def _get_slack_constraints(self):
        """Returns the constraints that the slack variables are positive.
        Called on initialization. """
        constraint = [self.v_slack > 0], [self.safety_slack > 0]

        return constraint

    def _get_pos_slack_constraints(self):
        """Returns the constraints that the position slack variable is positive. Called on
        initialization. """
        constraint = [self.pos_slack > 0]

        return constraint

    def solve_mpc(self, vopt, x0, current_time=None, preceding_timestamps=None,
                  preceding_velocities=None, preceding_positions=None):
        """Solves the MPC problem.
        Computes reference trajectories from optimal speed profile and
        current state. If the vehicle is a follower it also computes the state reference from the
        timegap. Updates the constraints and cost and solves the problem. """
        self.x0 = x0
        xref, uref = self._get_x_u_references(x0[1], vopt)

        if not (self.is_leader or
                current_time is None or
                preceding_timestamps is None or
                preceding_velocities is None or
                preceding_positions is None):

            xgapref = self._get_xgap_ref(current_time, preceding_timestamps, preceding_velocities,
                                         preceding_positions)
            self._update_safety_constraints(current_time, preceding_timestamps,
                                            preceding_positions)

            cost = self._get_mpc_cost(xref, uref, xgapref)
        else:
            cost = self._get_mpc_cost(xref, uref)

        self._update_x0_constraint(x0)

        self.prob.objective = cvxpy.Minimize(cost)

        try:
            self.prob.solve(solver='ECOS')
            self.status = 'OK'
            self.iterations += 1
            self.solver_iterations += self.prob.solver_stats.num_iters
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(self.prob.status))
            self.status = 'Could not solve MPC'

    def _update_x0_constraint(self, x0):
        """Updates the first dynamics constraint x(0) = x0. """
        self.prob.constraints[4] = self._get_x0_constraint(x0)[0]

    def _update_safety_constraints(self, current_time, preceding_timestamps, preceding_positions):
        """Updates the safety constraint. """
        if not self.is_leader:
            self.prob.constraints[8] = self._get_safety_constraints(current_time,
                                                                    preceding_timestamps,
                                                                    preceding_positions)[0]

    def _get_x_u_references(self, s0, v_opt):
        """Computes the different reference signals. """
        pos_ref = self._get_pos_ref(s0, v_opt)
        vel_ref = self._get_vel_ref(v_opt, pos_ref)
        acc_ref = self._get_acc_ref(vel_ref)

        xref = self._interleave_vectors(vel_ref, pos_ref)
        uref = acc_ref[:]

        return xref, uref

    def _get_pos_ref(self, s0, v_opt):
        """Computes the position reference trajectory. """
        pos_ref = numpy.zeros(self.h + 1)
        pos_ref[0] = s0

        for i in range(1, self.h + 1):
            pos_ref[i] = pos_ref[i - 1] + self.dt*v_opt.get_speed_at(pos_ref[i - 1])

        return pos_ref

    def _get_vel_ref(self, v_opt, pos_ref):
        """Computes the velocity reference trajectory. """
        vel_ref = v_opt.get_speed_at(pos_ref)

        return vel_ref

    def _get_acc_ref(self, vel_ref):
        """Computes the acceleration reference trajectory. """
        acc_ref = (vel_ref[1:] - vel_ref[:-1])/self.dt

        return acc_ref

    def _get_mpc_cost(self, xref, uref, xgapref=None):
        """Returns the mpc cost for the current iteration. """
        state_reference_cost = 0.5*cvxpy.quad_form(self.x, self.state_P) + \
                               self.state_Q_diag.dot(xref) * self.x

        input_reference_cost = 0.5*cvxpy.quad_form(self.u, self.input_P) + \
                               self.R_diag.dot(uref) * self.u

        cost = state_reference_cost + input_reference_cost + self.v_slack_cost + self.pos_slack_cost

        if not self.is_leader and xgapref is not None:

            cost = cost + self.safety_slack_cost

            if numpy.mean(xgapref[0::2]) > 0.1:
                timegap_cost = 0.5 * cvxpy.quad_form(self.x, self.timegap_P) + \
                               self.timegap_Q_diag.dot(xgapref) * self.x
                cost += timegap_cost

        return cost

    def _get_xgap_ref(self, current_time, timestamps, velocities, positions):
        """Returns the reference state for tracking the timegap of the preceding vehicle. """
        target_time = current_time - self.timegap + numpy.arange(self.h + 1)*self.dt + self.dt

        gap_vel = numpy.interp(target_time, timestamps, velocities)
        gap_pos = numpy.interp(target_time, timestamps, positions)

        xgapref = self._interleave_vectors(gap_vel, gap_pos)

        return xgapref

    def _get_v_slack_cost(self, cost_factor):
        """Returns the cost function for the velocity slack variable. The cost is quadratic.
        Called on initialization. """
        v_slack_P = numpy.eye(self.h + 1) * cost_factor

        cost = cvxpy.quad_form(self.v_slack, v_slack_P)

        return cost

    def _get_pos_slack_cost(self, cost_factor):
        """Returns the cost function for the position slack variable. The cost is quadratic.
        Called on initialization. """
        pos_slack_P = numpy.eye(self.h + 1) * cost_factor

        cost = cvxpy.quad_form(self.pos_slack, pos_slack_P)

        return cost

    def _get_safety_slack_cost(self, cost_factor):
        """Returns the cost function for the safety distance slack variable. The cost is quadratic.
        Called on initialization. """
        if not self.is_leader:
            safety_slack_P = numpy.eye(self.h + 1) * cost_factor

            cost = cvxpy.quad_form(self.safety_slack, safety_slack_P)
        else:
            cost = 0

        return cost

    def get_instantaneous_acceleration(self):
        """Returns the optimal acceleration for the current time instant.
        Returns 0 if none available. """
        try:
            return self.get_input_trajectory()[0]
        except TypeError:
            return 0

    def get_input_trajectory(self):
        """Returns the optimal input trajectory computed by the MPC. Returns an array of zeros if
        there is no optimal input trajectory available. """
        try:
            trajectory = numpy.array(self.u.value[:]).flatten()
            return trajectory
        except TypeError:
            print('MPC returning acc = 0')
            return numpy.zeros(self.h*self.nu)

    def get_predicted_states(self):
        """Returns the predicted velocity and position from the MPC, starting at x0. If there is
        no trajectories available a backup state is returned, which assumes zero acceleration. """
        try:
            return numpy.squeeze(numpy.asarray(self.x.value[0::2].flatten())), \
                   numpy.squeeze(numpy.asarray(self.x.value[1::2].flatten()))
        except TypeError:
            print('No optimal, returning backup predicted state. ')
            return self._get_backup_predicted_state()

    def _get_backup_predicted_state(self):
        """Returns a predicted state trajectory over one horizon. Used if there
        is no optimal trajectory available from the MPC solver. """
        vel = numpy.ones(self.h + 1)*self.x0[0]
        pos = self.x0[1] + self.x0[0]*self.dt*numpy.arange(self.h + 1)

        return vel, pos

    def add_position_constraint(self, position):
        """Adds a constraint for maximum position. """
        AX = sparse.kron(sparse.eye(self.h + 1), [0, 1])

        constraint = [AX*self.x - self.pos_slack <= position]

        self.prob.constraints += constraint

    @staticmethod
    def _interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c


def print_numpy(a):
    s = '['
    for v in a.flatten():
        s += ' {:.2f}'.format(v)
    s += ' ]'

    print(s)























