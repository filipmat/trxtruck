#!/usr/bin/env python


"""
Class for simulating a trx vehicle. Receives input commands and sends position information.
"""


import rospy
import sys
import math

import trxmodel

from trucksim.msg import MocapState, PWM
from geometry_msgs.msg import Twist


class SimTrx(object):

    def __init__(self, vehicle_id, mocap_topic_name, control_topic_name,
                 x=None, u=None, frequency=20):

        self.trx = trxmodel.Trx(x=x[:], u=u[:], ID=vehicle_id)  # Vehicle model.

        self.yaw_rate = 0
        self.acceleration = 0
        self.radius = 0

        self.dt = 1. / frequency  # Update interval.

        self.vehicle_id = vehicle_id

        # Initialize ROS node.
        rospy.init_node(vehicle_id + '_simulated', anonymous=False)

        # Subscriber for receiving speed control signal.
        rospy.Subscriber(control_topic_name, Twist, self._callback)

        # Publisher for publishing vehicle position and velocity.
        self.pub = rospy.Publisher(mocap_topic_name, MocapState, queue_size=10)

        # ROS update rate.
        self.update_rate = rospy.Rate(frequency)

        # Fix so that standing still at start by sending command to itself through trxvehicle.py.
        # Last command in trxvehicle.py is then set to the standstill values.
        self.pwm_start_pub = rospy.Publisher('pwm_commands', PWM, queue_size=1)

        self._initialize_standstill()

    def _initialize_standstill(self):
        """Publish PWM commands when starting so that the vehicle is standing still. """
        for i in range(5):
            self.update_rate.sleep()
            self.pwm_start_pub.publish(self.vehicle_id, 1500, 1500, 120)

        print('Trx test vehicle initialized, id: {}'.format(self.vehicle_id))

    def _callback(self, data):
        """Method called when subscriber receives data. Updates the input. """
        self.trx.set_throttle(data.linear.x)
        self.trx.set_steering(data.angular.z)

    def run(self):
        """Run the simulation. Moves the vehicle and publishes the position. """
        while not rospy.is_shutdown():
            self._move()

            self._publish_vehicle_state()

            self.update_rate.sleep()

    def _publish_vehicle_state(self):
        """Publishes the current vehicle state. """
        t = rospy.get_time()
        x = self.trx.get_x()
        self.pub.publish(t, self.vehicle_id, x[0], x[1], x[2], self.yaw_rate, x[3],
                         self.acceleration, self.radius)

    def _move(self):
        """Moves the vehicle as a unicycle and calculates velocity, yaw rate, etc. """

        last_x = self.trx.get_x()   # Save information for velocity calculation.

        self.trx.update(self.dt)    # Move vehicle.

        x = self.trx.get_x()

        # Calculate yaw rate.
        if x[2] < last_x[2] - math.pi:
            yaw_difference = x[2] - last_x[2] + 2 * math.pi
        elif x[2] > last_x[2] + math.pi:
            yaw_difference = x[2] - last_x[2] - 2 * math.pi
        else:
            yaw_difference = x[2] - last_x[2]
        self.yaw_rate = yaw_difference / self.dt

        # Calculate acceleration.
        self.acceleration = (x[3] - last_x[3]) / self.dt

        # Calculate turning radius.
        distance = math.sqrt((x[0] - last_x[0]) ** 2 + (x[1] - last_x[1]) ** 2)
        try:
            self.radius = distance / yaw_difference
        except ZeroDivisionError:
            self.radius = 0

    def __str__(self):
        """Returns a string with the vehicle id and the current state. """
        s = 'ID = {}: x = ['.format(self.vehicle_id)

        for x in self.trx.get_x():
            s += '{:.3f}, '.format(x)

        s = s[:-2]
        s += ']'

        return s


def main(args):
    """Creates a vehicle node and starts the simulation. The name of the vehicle is entered as an
    argument on the command line. The vehicle is initialized at the origin pointing to the left. """

    if len(args) < 1:
        print('Need to enter a vehicle ID.')
        sys.exit()

    vehicle_id = args[1]
    frequency = 20
    mocap_topic_name = 'mocap_state'
    control_topic_name = vehicle_id + '/cmd_vel'

    x = [0, 0, math.pi, 0]
    u = [0., 0.]

    simtrx = SimTrx(vehicle_id, mocap_topic_name, control_topic_name, x=x, u=u, frequency=frequency)

    simtrx.run()


if __name__ == '__main__':
    main(sys.argv)
