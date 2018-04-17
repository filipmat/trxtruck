#!/usr/bin/env python

"""
Simulate centralized MPC without using ROS.
"""

import sys
import numpy
import math
import time
import os

from matplotlib import pyplot

import path
import speed_profile
import frenetpid
import solver_distributed_mpc
import trxmodel
import helper

from trxtruck.msg import Recording


class DistributedMPC(object):

    def __init__(self, vehicles, vehicle_path, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap, k_p, k_i, k_d, simulation_length, xmin=None, xmax=None,
                 umin=None, umax=None, speed_ref=None):

        self.pwm_min = 1500
        self.pwm_max = 1990

        self.dt = delta_t
        self.iterations = int(simulation_length/self.dt)

        self.pt = vehicle_path

        self.timegap = timegap

        # Optimal speed profile in space. If none given, optimal speed is 1 m/s everywhere.
        if speed_ref is None:
            speed_ref = speed_profile.Speed([1], [1])
        self.original_speed_profile = speed_ref

        self.vehicles = vehicles
        self.n = len(vehicles)
        self.h = horizon

        # Matrices for storing control errors etc.
        self.timestamps = numpy.zeros((self.n, self.iterations))
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

        assumed_state_length = horizon + int(2.*timegap/delta_t)
        self.assumed_timestamps = numpy.zeros((self.n, assumed_state_length))
        self.assumed_velocities = numpy.zeros((self.n, assumed_state_length))
        self.assumed_positions = numpy.zeros((self.n, assumed_state_length))

        self.frenets = []           # Path tracking.
        self.path_positions = []    # Longitudinal path positions.
        self.mpcs = []
        self.speed_pwms = []        # Speed control signals.
        self.angle_pwms = []        # Wheel angle control signals.
        self.speed_profiles = []

        # Initialize lists.
        for i, vehicle in enumerate(self.vehicles):
            x = vehicle.get_x()
            self.frenets.append(frenetpid.FrenetPID(vehicle_path, k_p, k_i, k_d))
            self.path_positions.append(path.PathPosition(vehicle_path, [x[0], x[1]]))

            if i == 0:
                is_leader = True
            else:
                is_leader = False
            self.mpcs.append(solver_distributed_mpc.MPC(Ad, Bd, delta_t, horizon, zeta, Q, R,
                                                        truck_length, safety_distance, timegap,
                                                        xmin=xmin, xmax=xmax, umin=umin, umax=umax,
                                                        is_leader=is_leader))

            self.speed_pwms.append(1500)
            self.angle_pwms.append(1500)

            self.speed_profiles.append(speed_ref)

        self._order_follower_path_positions()

        for i in range(len(self.vehicles)):
            self.assumed_positions[i, :] = self.path_positions[i].get_position()

        self.k = 0

    def run(self):
        """Runs the simulation. """
        print('Simulation started. Simulated duration {:.2f}.'.format(self.dt*self.iterations))
        print('...')

        start_time = time.time()

        while self.k < self.iterations:
            self._control()

            if self.k % int(5./self.dt) == 0:
                print('Iteration {}/{}'.format(self.k, self.iterations))

            self.k += 1

            # if self.k == 300:
            #     self._brake(0)

        elapsed_time = time.time() - start_time
        average_time = elapsed_time / self.iterations

        print('Simulation completed. ')
        print('Elapsed time {:.2f}, average iteration time {:.3f}'.format(
            elapsed_time, average_time))
        print('Mean square path error = {:.5f}'.format(numpy.mean(numpy.square(self.path_errors))))
        print('Mean square vel error = {:.5f}'.format(
            numpy.mean(numpy.square(self.velocity_errors))))

    def _control(self):
        """Performs one control iteration. """

        # For each vehicle translate acceleration into speed control input. Get steering control
        # input from Frenet controller.
        for i, vehicle in enumerate(self.vehicles):
            # Get velocity from acceleration and velocity control input from vehicle model.
            x = vehicle.get_x()

            pos = self.path_positions[i].get_position()

            x0 = [x[3], pos]

            # Update current state.
            self.assumed_velocities[i, 0:-(self.h + 1)] = self.assumed_velocities[i, 1:-self.h]
            self.assumed_positions[i, 0:-(self.h + 1)] = self.assumed_positions[i, 1:-self.h]
            self.assumed_timestamps[i, 0:-(self.h + 1)] = self.assumed_timestamps[i, 1:-self.h]

            self.assumed_velocities[i, -(self.h + 1)] = x[3]
            self.assumed_positions[i, -(self.h + 1)] = pos
            self.assumed_timestamps[i, -(self.h + 1)] = self.k*self.dt

            # Solve MPC.
            if i == 0:
                self.mpcs[i].solve_mpc(self.speed_profiles[i], x0)
            else:
                self.mpcs[i].solve_mpc(self.speed_profiles[i], x0,
                                       self.assumed_timestamps[i, -(self.h + 1)],
                                       self.assumed_timestamps[i - 1],
                                       self.assumed_velocities[i - 1],
                                       self.assumed_positions[i - 1])

            velocities, positions = self.mpcs[i].get_predicted_states()

            # Update assumed state.
            self.assumed_velocities[i, -(self.h + 1):] = velocities
            self.assumed_positions[i, -(self.h + 1):] = positions
            self.assumed_timestamps[i, -(self.h + 1):] = \
                self.assumed_timestamps[i, -(self.h + 1)] + self.dt * numpy.arange(self.h + 1)

            # Get velocity control signal.
            acceleration = self.mpcs[i].get_instantaneous_acceleration()

            v = self._get_vel(i, acceleration)
            self.speed_pwms[i] = self._get_throttle_input(i, v)

            # Get angular velocity from Frenet controller and steering input from vehicle model.
            omega = self._get_omega(i, acceleration)
            self.angle_pwms[i] = trxmodel.angular_velocity_to_steering_input(omega, x[3])

            vehicle.update(self.dt, self.speed_pwms[i], self.angle_pwms[i])
            x = vehicle.get_x()
            self.path_positions[i].update_position([x[0], x[1]])

            # Store information.
            timegap = 0
            if i > 0 and vehicle.get_vel() != 0:
                timegap = (self.path_positions[i - 1].get_position() - pos) / x[3]

            self.timestamps[i, self.k] = self.assumed_timestamps[i, -(self.h + 1)]
            self.xx[i, self.k] = x[0]
            self.yy[i, self.k] = x[1]
            self.yaws[i, self.k] = x[2]
            self.positions[i, self.k] = pos
            self.velocities[i, self.k] = x[3]
            self.velocity_references[i, self.k] = self.speed_profiles[i].get_speed_at(pos)
            self.accelerations[i, self.k] = acceleration
            self.timegaps[i, self.k] = timegap
            self.path_errors[i, self.k] = self.frenets[i].get_y_error()
            self.velocity_errors[i, self.k] = self.speed_profiles[i].get_speed_at(pos) - x[3]
            self.speed_inputs[i, self.k] = self.speed_pwms[i]
            self.steering_inputs[i, self.k] = self.angle_pwms[i]

    def _get_omega(self, vehicle_index, acceleration):
        """Returns the control input omega for the specified vehicle. """
        pose = self.vehicles[vehicle_index].get_x()

        omega = self.frenets[vehicle_index].get_omega(pose[0], pose[1], pose[2], pose[3])

        return omega

    def _get_vel(self, vehicle_index, acceleration):
        """Returns the new target velocity from the acceleration and current control signal. """
        vel = self.vehicles[vehicle_index].get_vel() + acceleration * self.dt

        return vel

    def _get_throttle_input(self, vehicle_index, new_vel):
        """Returns the new control input for the vehicle. """
        pwm_diff = trxmodel.linear_velocity_to_throttle_input(new_vel) - \
                   trxmodel.linear_velocity_to_throttle_input(
                       self.vehicles[vehicle_index].get_vel())
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

    def _brake(self, vehicle_index):
        """Brakes the vehicle. """
        self.speed_profiles[vehicle_index] = speed_profile.Speed([1], [0])

    def _unbrake(self, vehicle_index):
        """Undoes the braking of the vehicle. """
        self.speed_profiles[vehicle_index] = speed_profile.Speed([1], [0])

    def plot_stuff(self):
        x = [item[0] for item in self.pt.path]
        y = [item[1] for item in self.pt.path]

        pyplot.figure(figsize=(10, 10))

        ax = pyplot.subplot(231)
        ax.set_ylim([0, self.timegap*2])
        ax.set_title('Timegap')
        ax.set_xlabel('s')
        ax.set_ylabel('gap')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.timegaps[i], label=self.vehicles[i].ID)
        pyplot.plot(self.timestamps[i], numpy.ones(len(self.timestamps[i]))*self.timegap,
                    label='reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(232)
        ax.set_title('Speed space')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.positions[i], self.velocities[i], label=self.vehicles[i].ID)
        voptend = numpy.argmax(self.original_speed_profile.pos > numpy.max(self.positions))
        if voptend == 0:
            voptend = len(self.original_speed_profile.pos)
        pyplot.plot(self.original_speed_profile.pos[:voptend],
                    self.original_speed_profile.vel[:voptend], label='reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(233)
        ax.set_title('Speed time')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.velocities[i], label=self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(234)
        ax.set_title('Path')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.positions[i], label=self.vehicles[i].ID)
        # pyplot.plot(x, y, label='Reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(235)
        ax.set_title('Acc')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.timestamps[i], self.accelerations[i], label=self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(236)
        ax.set_title('Gap')
        for i in range(len(self.vehicles)):
            if i > 0:
                pyplot.plot(self.timestamps[i], self.positions[i - 1] - self.positions[i],
                            label=self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        pyplot.tight_layout(pad=0.5, w_pad=0.5, h_pad=2)
        pyplot.show()

    def save_data(self, filename):
        """Save data to file. """

        name = self._get_filename(filename, '.txt')

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

    @staticmethod
    def _get_filename(prefix, suffix, padding=2):
        """Sets a filename on the form filename_prefixZ.bag where Z is the first free number.
        Pads with zeros, e.g. first free number 43 and padding=5 will give 00043. """
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

        i = 0
        while os.path.exists(os.path.join(__location__, '{}{}{}'.format(
                prefix, str(i).zfill(padding), suffix))):
            i += 1

        filename = os.path.join(__location__, '{}{}{}'.format(
            prefix, str(i).zfill(padding), suffix))

        return filename


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

    horizon = 10
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.8
    Q_v = 1  # Part of Q matrix for velocity tracking.
    Q_s = 1  # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2)  # State tracking.
    R_acc = 0.1
    R = numpy.array([1]) * R_acc  # Input tracking.
    velocity_min = 0.
    velocity_max = 1.5
    position_min = -100000.
    position_max = 1000000.
    acceleration_min = -0.5
    acceleration_max = 0.5
    truck_length = 0.3
    safety_distance = 0.2
    timegap = 1.

    simulation_length = 20  # How many seconds to simulate.

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
    # vopt = speed_profile.Speed([1], [1])

    # Controller reference path.
    x_radius = 1.4
    y_radius = 1.2
    center = [0.2, -y_radius / 2]
    pts = 400

    save_data = True
    filename = 'sim_dmpc' + '_' + '_'.join(vehicle_ids) + '_'

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
        # # x = [0, 0, math.pi, 0]
        vehicles.append(trxmodel.Trx(x=x, ID=vehicle_id))

    mpc = DistributedMPC(vehicles, pt, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                         safety_distance, timegap, k_p, k_i, k_d, simulation_length,
                         xmin=xmin, xmax=xmax, umin=umin, umax=umax, speed_ref=speed_ref)

    mpc.run()

    if save_data:
        mpc.save_data_as_rosbag(filename)

    mpc.plot_stuff()


if __name__ == '__main__':
    main(sys.argv)