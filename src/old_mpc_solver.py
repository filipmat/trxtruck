import numpy
import cvxpy
import scipy.sparse as sparse


def print_numpy(a):
    s = '['
    for v in a.flatten():
        s += ' {:.2f}'.format(v)
    s += ' ]'

    print(s)


class TruckMPC(object):

    def __init__(self, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None,
                 vehicle_id=None, saved_h=2, is_leader=False):

        self.Ad = Ad
        self.Bd = Bd
        self.dt = delta_t
        self.h = horizon
        self.zeta = zeta
        self.Q = Q
        self.R = R
        self.truck_length = truck_length
        self.safety_distance = safety_distance
        self.timegap = timegap

        if vehicle_id is None:
            self.vehicle_id = 'NO_ID'
        else:
            self.vehicle_id = vehicle_id

        self.nx = self.Ad.shape[0]
        self.nu = self.Bd.shape[1]

        self.x = cvxpy.Variable((self.h + 1) * self.nx)
        self.u = cvxpy.Variable(self.h * self.nu)

        self.inf = 1000000  # "Infinity" used when there are not limits give.

        self.is_leader = is_leader

        self.status = 'OK'  # Status of solver.

        if xmin is None:
            self.xmin = -self.inf*numpy.ones(self.nx)  # No min state limit.
        else:
            self.xmin = xmin

        if xmax is None:
            self.xmax = self.inf*numpy.ones(self.nx)  # No max state limit.
        else:
            self.xmax = xmax

        if umin is None:
            self.umin = -self.inf*numpy.ones(self.nu)  # No min input limit.
        else:
            self.umin = umin

        if umax is None:
            self.umax = self.inf*numpy.ones(self.nu)  # No max input limit.
        else:
            self.umax = umax

        if x0 is None:
            self.x0 = numpy.zeros(self.nx)  # Origin default initial condition.
        else:
            self.x0 = x0

        # Computed from optimal speed profile.
        self.pos_ref = numpy.zeros(self.h + 1)
        self.vel_ref = numpy.zeros(self.h + 1)
        self.acc_ref = numpy.zeros(self.h)

        # Put together from reference trajectories above.
        self.xref = numpy.zeros((self.h + 1)*self.nx)
        self.uref = numpy.zeros(self.h*self.nu)

        # Assumed state contains past states and optimal state from MPC.
        self.saved_h = saved_h
        self.assumed_x = numpy.zeros(self.h * (self.saved_h + 1) * self.nx)

        # State of preceding vehicle.
        self.preceding_x = numpy.zeros(self.h * (self.saved_h + 1) * self.nx)

        # Problem
        self.prob = cvxpy.Problem(cvxpy.Minimize(1))

        state_constraint1, state_constraint2 = self.get_state_constraints() # Fixed.
        input_constraint1, input_constraint2 = self.get_input_constraints() # Fixed.
        dynamics_constraints1 = self.get_dynamics_constraints_part_one()    # Update during mpc.
        dynamics_constraints2 = self.get_dynamics_constraints_part_two()    # Fixed.

        self.prob.constraints += state_constraint1
        self.prob.constraints += state_constraint2
        self.prob.constraints += input_constraint1
        self.prob.constraints += input_constraint2
        self.prob.constraints += dynamics_constraints1  # update
        self.prob.constraints += dynamics_constraints2
        if not self.is_leader:
            self.prob.constraints += self.get_safety_constraints() # update

        # Cholesky cost calculation.
        self.PQ_ch_s = numpy.linalg.cholesky(
            numpy.kron(numpy.eye(self.h + 1), (1 - self.zeta) * self.Q))
        self.PQ_ch_t = numpy.linalg.cholesky(
            numpy.kron(numpy.eye(self.h + 1), self.zeta * self.Q))
        self.PR_ch = numpy.linalg.cholesky(
            numpy.kron(numpy.eye(self.h), self.R))

        # Quad form cost calculation.
        self.state_P = self.get_state_P()
        self.input_P = self.get_input_P()
        self.timegap_P = self.get_timegap_P()

        self.state_Q_diag = sparse.kron(sparse.eye(self.h + 1), -(1 - self.zeta)*self.Q)
        self.timegap_Q_diag = sparse.kron(sparse.eye(self.h + 1), -self.zeta * self.Q)
        self.R_diag = sparse.kron(sparse.eye(self.h), -self.R)

        self.timegap_shift = int(round(self.timegap / self.dt))

    def get_state_constraints(self):
        """Returns the constraints corresponding to state limits. """
        AX = sparse.eye((self.h + 1) * self.nx)

        upper = numpy.tile(self.xmax, self.h + 1)
        lower = numpy.tile(self.xmin, self.h + 1)

        return [AX * self.x <= upper], [AX * self.x >= lower]

    def get_input_constraints(self):
        """Returns the constraints corrseponding to input limits. """
        AU = numpy.eye(self.h*self.nu)
        upper = numpy.tile(self.umax, self.h)
        lower = numpy.tile(self.umin, self.h)

        return [AU * self.u <= upper], [AU * self.u >= lower]

    def get_dynamics_constraints_part_one(self):
        AA = sparse.hstack([sparse.eye(self.nx), sparse.csc_matrix((self.nx, self.h*self.nx))])

        constraint = [AA * self.x == self.x0]

        return constraint

    def get_dynamics_constraints_part_two(self):
        """Precomputes the parts of the dynamics that can be precomputed. """
        AA = sparse.kron(sparse.hstack([sparse.eye(self.h), sparse.csc_matrix((self.h, 1))]),
                         self.Ad) + \
             sparse.kron(sparse.hstack([sparse.csc_matrix((self.h, 1)), sparse.eye(self.h)]),
                        -sparse.eye(self.nx))

        BB = sparse.kron(sparse.eye(self.h), self.Bd)

        constraint = [AA * self.x + BB * self.u == 0]

        return constraint

    def get_safety_constraints(self):
        """Returns the constraints corresponding to keeping a safe distance."""
        pos = self.preceding_x[
              (self.h*self.saved_h - 1) * self.nx:
              (self.h*(self.saved_h + 1)) * self.nx][1::2]
        posmax = pos - self.truck_length - self.safety_distance
        vmax = numpy.ones(self.h + 1)*self.inf

        statemax = self.interleave_vectors(vmax, posmax)

        AS = sparse.eye((self.h + 1)*self.nx)

        constraints = [AS * self.x <= statemax]

        return constraints

    def update_mpc_constraints(self):
        self.prob.constraints[4] = self.get_dynamics_constraints_part_one()[0]
        if not self.is_leader:
            self.prob.constraints[6] = self.get_safety_constraints()[0]

    def set_new_x0(self, x0):
        """Sets the current initial value. The positions of previous saved
        states are shifted. """
        self.x0 = x0
        self.assumed_x[:self.h * self.saved_h * self.nx] = \
            self.assumed_x[self.nx:(self.h*self.saved_h + 1) * self.nx]
        self.assumed_x[self.saved_h * self.h * self.nx] = x0[0]
        self.assumed_x[self.saved_h * self.h * self.nx + 1] = x0[1]

    def compute_optimal_trajectories(self, vopt, preceding_x=None):
        """Computes the optimal trajectories using MPC and updates the vehicle
        assumed state. """
        self.update_mpc(vopt, preceding_x)
        self.solve_mpc()
        self.update_assumed_state()

    def update_mpc(self, vopt, preceding_x):
        """Updates the MPC problem with the new initial position, optimal
        speed trajectory and preceding vehicle trajectory. """
        if preceding_x is not None:
            self.preceding_x = preceding_x
        self.compute_references(self.x0[1], vopt)

    def solve_mpc(self):
        """Solves the MPC problem. """

        #cost = self.get_mpc_cost_cholesky()
        cost = self.get_mpc_cost_quad_form()

        self.prob.objective = cvxpy.Minimize(cost)

        self.update_mpc_constraints()

        try:
            self.prob.solve(solver='ECOS')
            self.status = 'OK'
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(self.prob.status))
            self.status = 'Could not solve MPC'

    def update_assumed_state(self):
        """Updates the assumed state. The length is self.saved_h horizons. The
        first part are the previous states, the second part starts with the
        current state and then contains the optimal state from the MPC solution.
        """
        try:
            self.assumed_x[self.h*self.saved_h*self.nx:] = \
                self.x.value[:self.h*self.nx].flatten()
        except TypeError:
            self.assumed_x[self.h*self.saved_h*self.nx:] = \
                self.get_backup_predicted_state()
            self.status = 'No MPC optimal. '

    def get_backup_predicted_state(self):
        """Returns a predicted state trajectory over one horizon. Used if there
        is no optimal trajectory available from the MPC solver. """
        state = numpy.zeros(self.h*self.nx)
        state[0::2] = self.x0[0]
        state[1::2] = self.x0[1] + self.x0[0]*self.dt*numpy.arange(self.h)

        return state

    def compute_references(self, s0, v_opt):
        """Computes the different reference signals. """
        self.compute_pos_ref(s0, v_opt)
        self.compute_vel_ref(v_opt)
        self.compute_acc_ref()

        self.xref = self.interleave_vectors(self.vel_ref, self.pos_ref)
        self.uref = self.acc_ref[:]

    def compute_pos_ref(self, s0, v_opt):
        """Computes the position reference trajectory. """
        self.pos_ref[0] = s0

        for i in range(1, self.h + 1):
            self.pos_ref[i] = self.pos_ref[i - 1] + \
                              self.dt*v_opt.get_speed_at(self.pos_ref[i - 1])

    def compute_vel_ref(self, v_opt):
        """Computes the velocity reference trajectory. """
        self.vel_ref = v_opt.get_speed_at(self.pos_ref)

    def compute_acc_ref(self):
        """Computes the acceleration reference trajectory. """
        self.acc_ref = (self.vel_ref[1:] - self.vel_ref[:-1])/self.dt

    def get_assumed_state(self):
        """Returns the assumed state. """
        return self.assumed_x[:]

    def get_instantaneous_acceleration(self):
        """Returns the optimal acceleration for the current time instant.
        Returns 0 if none available. """
        try:
            return self.get_input_trajectory()[0]
        except TypeError:
            return 0

    def get_input_trajectory(self):
        """Returns the optimal input trajectory computed by the MPC. Returns
        an array of zeros if there is no optimal input trajectory available. """
        try:
            u_copy = self.u.value[:]
            trajectory = numpy.array(u_copy).flatten()
            return trajectory
        except TypeError:
            return numpy.zeros(self.h*self.nu)

    @staticmethod
    def interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c

    def set_leader(self, is_leader):
        """Set the vehicle to be the leader or not. Default is false. """
        self.is_leader = is_leader

    # Cholesky cost calculation.

    def get_mpc_cost_cholesky(self):
        """Returns the cost for the MPC problem. The cost is the combined cost
        for tracking reference, timegap and input trajectories. If the vehicle
        is the leader the timegap tracking cost is excluded. """
        cost = self.get_state_reference_cost() + self.get_input_reference_cost()

        if not self.is_leader:
            gap_cost = self.get_timegap_cost()
            cost = cost + gap_cost

        return cost

    def get_state_reference_cost(self):
        """Returns the cost corresponding to state reference tracking. """
        cost = cvxpy.sum_entries(cvxpy.square(self.PQ_ch_s * (self.x - self.xref)))

        return cost

    def get_input_reference_cost(self):
        """Returns the cost corresponding to input reference tracking. """
        cost = cvxpy.sum_entries(cvxpy.square(self.PR_ch * (self.u - self.uref)))

        return cost

    def get_timegap_cost(self):
        """Returns the cost corresponding to timegap tracking. """
        shift = int(round(self.timegap / self.dt))
        xgapref = self.preceding_x[
                       (self.h*self.saved_h - shift)*self.nx:
                       (self.h*(self.saved_h + 1) - shift + 1)*self.nx]

        cost = cvxpy.sum_entries(cvxpy.square(self.PQ_ch_t * (self.x - xgapref)))

        return cost

    # Quad form cost calculation.

    def get_state_P(self):
        """Used at initialization. """
        P = sparse.kron(sparse.eye(self.h + 1), (1 - self.zeta)*self.Q)

        return P

    def get_input_P(self):
        """Used at initialization."""
        P = sparse.kron(sparse.eye(self.h), self.R)

        return P

    def get_timegap_P(self):
        """Used at initialization. """
        P = sparse.kron(sparse.eye(self.h + 1), self.zeta*self.Q)

        return P

    def get_mpc_cost_quad_form(self):
        cost = 0.5*cvxpy.quad_form(self.x, self.state_P) + \
               self.state_Q_diag.dot(self.xref)*self.x + \
               0.5*cvxpy.quad_form(self.u, self.input_P) + self.R_diag.dot(self.acc_ref)*self.u

        if not self.is_leader:
            xgapref = self.preceding_x[
                      (self.h * self.saved_h - self.timegap_shift) * self.nx:
                      (self.h * (self.saved_h + 1) - self.timegap_shift + 1) * self.nx]
            timegap_q = self.timegap_Q_diag.dot(xgapref)

            cost = cost + 0.5*cvxpy.quad_form(self.x, self.timegap_P) + timegap_q*self.x

        return cost


def main():
    pass

if __name__ == '__main__':
    main()


























