#!/usr/bin/env python

"""
Simulate centralized MPC without using ROS.
"""

import sys
import numpy
import math
import time
import random

from matplotlib import pyplot

import path
import speed_profile
import frenetpid
import solver_centralized_mpc
import trxmodel
import helper

from trxtruck.msg import Recording


class CentralizedMPC(object):

    def __init__(self, vehicles, vehicle_path, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap, k_p, k_i, k_d, simulation_length, xmin=None, xmax=None,
                 umin=None, umax=None, speed_ref=None, delay=0., variance=0.):

        self.dt = delta_t
        self.h = horizon
        self.iterations = int(simulation_length/self.dt)

        self.pt = vehicle_path

        self.variance = variance

        self.timegap = timegap

        # Optimal speed profile in space. If none given, optimal speed is 1 m/s everywhere.
        if speed_ref is None:
            speed_ref = speed_profile.Speed([1], [1])
        self.speed_profile = speed_ref
        self.original_speed_profile = speed_ref

        self.pwm_max = 1990
        self.pwm_min = 1500

        self.mpc = solver_centralized_mpc.MPC(len(vehicles), Ad, Bd, delta_t, horizon, zeta, Q, R,
                                              truck_length, safety_distance, timegap, xmin, xmax,
                                              umin, umax)
        self.vehicles = vehicles
        self.n = len(vehicles)

        # Matrices for storing control errors etc.
        self.timestamps = numpy.kron(numpy.ones((self.n, 1)), self.dt*numpy.arange(self.iterations))
        self.xx = numpy.zeros((self.n, self.iterations))
        self.yy = numpy.zeros((self.n, self.iterations))
        self.yaws = numpy.zeros((self.n, self.iterations))
        self.positions = numpy.zeros((self.n, self.iterations))
        self.velocities = numpy.zeros((self.n, self.iterations))
        self.velocity_references = numpy.zeros((self.n, self.iterations))
        self.accelerations = numpy.zeros((self.n, self.iterations))
        self.timegaps = numpy.zeros((self.n, self.iterations))
        self.path_errors = numpy.zeros((self.n, self.iterations))
        self.velocity_errors = numpy.zeros((self.n, self.iterations))
        self.speed_inputs = numpy.zeros((self.n, self.iterations))
        self.steering_inputs = numpy.zeros((self.n, self.iterations))

        # Save old initial positions used by MPC and path tracking to simulate communication delay.
        self.saved_num = int(math.ceil(delay/self.dt)) + 1
        self.delay_counter = 0
        self.delayed_states = numpy.zeros((self.saved_num, len(self.vehicles), 4))

        self.frenets = []           # Path tracking.
        self.path_positions = []    # Longitudinal path positions.
        self.speed_pwms = []        # Speed control signals.
        self.angle_pwms = []        # Wheel angle control signals.

        # Initialize lists.
        for i, vehicle in enumerate(self.vehicles):
            x = vehicle.get_x()
            self.frenets.append(frenetpid.FrenetPID(vehicle_path, k_p, k_i, k_d))
            self.path_positions.append(path.PathPosition(vehicle_path, [x[0], x[1]]))
            self.speed_pwms.append(1500)
            self.angle_pwms.append(1500)
            self.delayed_states[:, i, :] = x

        self._order_follower_path_positions()

        self.k = 0

        self.mpctime = 0

    def run(self):
        """Runs the simulation. """
        print('Simulation started. Simulated duration {:.2f}.'.format(self.dt*self.iterations))
        print('Horizon = {:.2f}, vehicles = {}.'.format(self.h, len(self.vehicles)))
        print('...')

        start_time = time.time()

        while self.k < self.iterations:

            # Store the current state at the index containing the oldest saved state.
            for i, vehicle in enumerate(self.vehicles):
                x = vehicle.get_x()
                self.delayed_states[self.delay_counter, i, :] = x

            # Increment counter.
            self.delay_counter += 1
            if self.delay_counter >= self.saved_num:
                self.delay_counter = 0

            self._control()
            if self.k % int(5./self.dt) == 0:
                print('Iteration {}/{}'.format(self.k, self.iterations))
            self.k += 1

            # if self.k == 300:
            #     self._brake()

            # if self.k == 300:
            #     maxpos = self.path_positions[0].get_position() + 5
            #     print('Added constraint pos < {:.3f}'.format(maxpos))
            #     self.mpc.add_position_constraint(maxpos)

        elapsed_time = time.time() - start_time
        average_iteration_time = elapsed_time / self.iterations
        average_mpc_time = self.mpctime / self.iterations
        average_solver_iterations = self.mpc.solver_iterations / self.mpc.iterations

        print('Simulation completed. ')
        print('Elapsed time {:.2f}, average iteration time {:.4f}'.format(
            elapsed_time, average_iteration_time))
        print('Average MPC time {:.4f}, average solver iterations {:.2f}'.format(
            average_mpc_time, average_solver_iterations))
        print('Mean square path error = {:.5f}'.format(numpy.mean(numpy.square(self.path_errors))))
        print('Mean square vel error = {:.5f}'.format(
            numpy.mean(numpy.square(self.velocity_errors))))

    def _control(self):
        """Performs one control iteration. """

        x0s = self._get_x0s()  # Get initial conditions.

        tt = time.time()
        self.mpc.solve_mpc(self.speed_profile, x0s)  # Solve MPC problem.
        self.mpctime += time.time() - tt

        accelerations = self.mpc.get_instantaneous_accelerations()  # Get accelerations.

        # For each vehicle translate acceleration into speed control input. Get steering control
        # input from Frenet controller.
        for i, vehicle in enumerate(self.vehicles):
            # Get velocity from acceleration and velocity control input from vehicle model.
            x = self._get_delayed_x(i)

            v = self._get_vel(i, accelerations[i])
            self.speed_pwms[i] = self._get_throttle_input(i, v)

            # Get angular velocity from Frenet controller and steering input from vehicle model.
            omega = self._get_omega(i, accelerations[i])
            # TODO: check which variant of v to use.
            self.angle_pwms[i] = trxmodel.angular_velocity_to_steering_input(omega, x[3])

            vehicle.update(self.dt, self.speed_pwms[i], self.angle_pwms[i])
            self.path_positions[i].update_position([x[0], x[1]])

            # Store information.
            pos = self.path_positions[i].get_position()

            timegap = 0
            if i > 0 and x[3] != 0:
                timegap = (self.path_positions[i - 1].get_position() - pos) / x[3]

            self.xx[i, self.k] = x[0]
            self.yy[i, self.k] = x[1]
            self.yaws[i, self.k] = x[2]
            self.positions[i, self.k] = pos
            self.velocities[i, self.k] = x[3]
            self.velocity_references[i, self.k] = self.speed_profile.get_speed_at(pos)
            self.accelerations[i, self.k] = accelerations[i]
            self.timegaps[i, self.k] = timegap
            self.path_errors[i, self.k] = self.frenets[i].get_y_error()
            self.velocity_errors[i, self.k] = self.speed_profile.get_speed_at(pos) - x[3]
            self.speed_inputs[i, self.k] = self.speed_pwms[i]
            self.steering_inputs[i, self.k] = self.angle_pwms[i]

    def _get_delayed_x(self, vehicle_index):
        """Returns the delayed state for the vehicle. """
        return self.delayed_states[self.delay_counter, vehicle_index, :]

    def _get_x0s(self):
        """Returns a stacked vector with the initial condition x0 = [v0, s0] for each vehicle. """
        x0s = numpy.zeros(2*len(self.vehicles))
        for i, vehicle in enumerate(self.vehicles):
            x = self._get_delayed_x(i)
            x0s[i*2] = x[3] + random.gauss(0, self.variance)
            x0s[i*2 + 1] = self.path_positions[i].get_position() + random.gauss(0, self.variance)

        return x0s

    def _get_omega(self, vehicle_index, acceleration):
        """Returns the control input omega for the specified vehicle. """
        # TODO: check which variant of speed measurement gives correct path tracking.
        pose = self._get_delayed_x(vehicle_index)

        v = pose[3]
        # v = trxmodel.throttle_input_to_linear_velocity(self.speed_pwms[vehicle_index])
        # v = pose[3] + self.dt * acceleration

        omega = self.frenets[vehicle_index].get_omega(pose[0], pose[1], pose[2], v)

        return omega

    def _get_vel(self, vehicle_index, acceleration):
        """Returns the new target velocity from the acceleration and current control signal. """
        x = self._get_delayed_x(vehicle_index)
        vel = x[3] + acceleration * self.dt

        return vel

    def _get_throttle_input(self, vehicle_index, new_vel):
        """Returns the new control input for the vehicle. """
        x = self._get_delayed_x(vehicle_index)

        # pwm_diff = trxmodel.linear_velocity_to_throttle_input(new_vel) - \
        #            trxmodel.linear_velocity_to_throttle_input(
        #                self.vehicles[vehicle_index].get_vel())
        pwm_diff = trxmodel.linear_velocity_to_throttle_input(new_vel) - \
                   trxmodel.linear_velocity_to_throttle_input(x[3])
        speed_pwm = self.speed_pwms[vehicle_index] + pwm_diff

        if speed_pwm > self.pwm_max:
            speed_pwm = self.pwm_max
        if speed_pwm < self.pwm_min:
            speed_pwm = self.pwm_min

        return speed_pwm

    def _order_follower_path_positions(self):
        """Fixes the path longitudinal positions so that each follower vehicle is positioned behind
        the preceding vehicle. """
        for i, vehicle in enumerate(self.vehicles):

            if i > 0:
                preceding_position = self.path_positions[i - 1].get_position()

                if (self.path_positions[i].get_position() - 0.01 > preceding_position or
                        self.path_positions[i].get_position() + 0.01 < preceding_position):

                    self.path_positions[i].set_position_behind(preceding_position)

    def _brake(self):
        """Brakes all vehicles. """
        self.speed_profile = speed_profile.Speed([1], [0])

    def unbrake(self):
        """Undoes the braking. """
        self.speed_profile = self.original_speed_profile

    def plot_stuff(self):
        x = [item[0] for item in self.pt.path]
        y = [item[1] for item in self.pt.path]

        pyplot.figure(figsize=(10, 10))

        ax = pyplot.subplot(321)
        ax.set_ylim([0, self.timegap*2])
        ax.set_title('Timegap')
        ax.set_xlabel('s')
        ax.set_ylabel('gap')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.timegaps[i], label=self.vehicles[i].ID)
        pyplot.plot(self.timestamps[0], numpy.ones(len(self.timestamps[0]))*self.timegap,
                    label='reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(322)
        ax.set_title('Speed space')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.positions[i], self.velocities[i], label=self.vehicles[i].ID)
        vref = self.original_speed_profile.get_speed_at(self.positions[0])
        pyplot.plot(self.positions[0], vref, label='reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(323)
        ax.set_title('Speed time')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.velocities[i], label=self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(324)
        ax.set_title('Path')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.xx[i], self.yy[i], label=self.vehicles[i].ID)
        pyplot.plot(x, y, label='Reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(325)
        ax.set_title('Acceleration')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.accelerations[i], label = self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(326)
        ax.set_title('Position')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.positions[i], label = self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        pyplot.tight_layout(pad=0.5, w_pad=0.5, h_pad=2)
        pyplot.show()

    def save_data(self, filename):
        """Save data to file. """

        name = helper.get_unique_filename(filename, '.txt')

        print('Saving data to {} ...'.format(name))

        header = 't'
        data = self.timestamps[:]

        for i in range(len(self.vehicles)):
            data = numpy.vstack([data, self.xx[i], self.yy[i], self.positions[i],
                                 self.velocities[i], self.accelerations[i], self.timegaps[i],
                                 self.path_errors[i], self.velocity_errors[i], self.speed_inputs[i],
                                 self.steering_inputs[i]])
            header += ',x,y,s,v,a,timegap,path_error,v_error,throttle,steering'

        data = data.T

        with open(name, 'w+') as datafile_id:
            numpy.savetxt(datafile_id, data, fmt='%.4f', header=header, delimiter=',')

    def save_data_as_rosbag(self, filename):
        """Save data to rosbag file. """
        recorder = helper.RosbagRecorder(filename, '.bag')
        recorder.start()

        print('Recording data to {} ...'.format(recorder.get_filename()))

        for j, vehicle in enumerate(self.vehicles):

            msg = Recording()
            msg.id = vehicle.ID
            for i in range(self.iterations):

                msg.time = self.timestamps[j, i]
                msg.x = self.xx[j, i]
                msg.y = self.yy[j, i]
                msg.yaw = self.yaws[j, i]
                msg.v = self.velocities[j, i]
                msg.pos = self.positions[j, i]
                msg.vref = self.velocity_references[j, i]
                msg.timegap = self.timegaps[j, i]
                msg.acc = self.accelerations[j, i]
                msg.path_error = self.path_errors[j, i]
                msg.velocity_input = int(self.speed_inputs[j, i])
                msg.steering_input = int(self.steering_inputs[j, i])
                msg.gear = 0

                recorder.write(msg, topic_name=vehicle.ID)

        recorder.stop()


def main(args):

    vehicle_amount = 1
    if len(args) > 1:
        vehicle_amount = int(args[1])

    vehicle_ids = []
    for i in range(vehicle_amount):
        vehicle_ids.append('v{}'.format(i + 1))

    # PID parameters for path tracking.
    k_p = 0.5
    k_i = -0.02
    k_d = 3

    horizon = 15
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.5
    Q_v = 1  # Part of Q matrix for velocity tracking.
    Q_s = 1  # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2)  # State tracking.
    R_acc = 0.1
    R = numpy.array([1]) * R_acc  # Input tracking.
    velocity_min = 0.
    velocity_max = 2.
    position_min = -100000.
    position_max = 1000000.
    acceleration_min = -1.5
    acceleration_max = 1.5
    truck_length = 0.3
    safety_distance = 0.2
    timegap = 1.

    delay = 0.3

    simulation_length = 40  # How many seconds to simulate.

    xmin = numpy.array([velocity_min, position_min])
    xmax = numpy.array([velocity_max, position_max])
    umin = numpy.array([acceleration_min])
    umax = numpy.array([acceleration_max])

    # Reference speed profile.
    opt_v_pts = 400  # How many points.
    opt_v_max = 1.2
    opt_v_min = 0.8
    opt_v_period_length = 60  # Period in meters.
    speed_ref = speed_profile.Speed()
    speed_ref.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    speed_ref.repeating = True

    # Controller reference path.
    x_radius = 1.4
    y_radius = 1.2
    center = [0.2, -y_radius / 2]
    pts = 400

    plot_data = True
    save_data = True
    filename = 'measurements/sim_cmpc' + '_' + '_'.join(vehicle_ids) + '_'

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    start_distance = 0.5
    path_len = pt.get_path_length()

    vehicles = []
    for i, vehicle_id in enumerate(vehicle_ids):

        theta = (len(vehicle_ids) - i - 1)*2*math.pi*start_distance/path_len + 0.1

        xcoord = center[0] + x_radius*math.cos(theta)
        ycoord = center[1] + y_radius*math.sin(theta)

        x = [xcoord, ycoord, theta + math.pi/2, 0]
        # x = [center[0], center[1] + y_radius, math.pi, 0]

        vehicles.append(trxmodel.Trx(x=x, ID=vehicle_id))

    mpc = CentralizedMPC(vehicles, pt, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                         safety_distance, timegap, k_p, k_i, k_d, simulation_length,
                         xmin=xmin, xmax=xmax, umin=umin, umax=umax, speed_ref=speed_ref,
                         delay=delay)

    mpc.run()

    if save_data:
        mpc.save_data_as_rosbag(filename)

    if plot_data:
        mpc.plot_stuff()


if __name__ == '__main__':
    main(sys.argv)