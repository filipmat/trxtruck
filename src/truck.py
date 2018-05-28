#!/usr/bin/env python

"""
Dispatch for old truck. Receives pwm commands, sends to truck with socket. 
"""

from socket import *
import struct
import time
import sys

import rospy

from trxtruck.msg import PWM


class Truck(object):

    def __init__(self, vehicle_id, address, pwm_topic_name, control_frequency=20):

        self.vehicle_id = vehicle_id
        self.address = address

        self.client_socket = socket(AF_INET, SOCK_DGRAM)
        self.client_socket.settimeout(0.1)
        self.seqNum = 0xFFFF

        self.velocity_command = 1500
        self.steering_command = 1500
        self.gear_command = 0

        self.packer = struct.Struct('<IIHhhh')

        self.print_info = False

        rospy.init_node('truck_' + self.vehicle_id, anonymous=False)

        rospy.Subscriber(pwm_topic_name, PWM, self.pwm_callback)

        self.control_rate = rospy.Rate(control_frequency)

        self._send_data(self.velocity_command, self.steering_command, True)

        self.run()

    def pwm_callback(self, data):
        if self.vehicle_id == data.id:
            self.steering_command = data.angle
            self.velocity_command = data.velocity
            self.gear_command = data.gear

    def run(self):
        """Send commands to vehicle at the specified frequency while ROS is running. """
        while not rospy.is_shutdown():
            self._send_data(self.velocity_command, self.steering_command)
            self.control_rate.sleep()

    def _send_data(self, speed, angle, first=False):
        """Sends speed, angle to truck truck_id. """
        # Get the address of the truck corresponding to the truck_id.

        # Probably not necessary stuff.
        if first:
            ms = 0xFFFFFFFF
            ns = 0xFFFFFFFF
        else:
            self.seqNum = (self.seqNum + 1) % 0xFFFF
            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10 ** 9))

        # Pack message and send to address.
        command_msg = self.packer.pack(*(ms, ns, self.seqNum, speed, angle, 60))

        self.client_socket.sendto(command_msg, self.address)

        # Print info if enabled.
        if self.print_info:
            print('Sending [speed {:.0f}, angle {:.0f}] to {}'.format(
                speed, angle, self.address))


def main(args):

    address = ('192.168.1.194', 2390)
    vehicle_id = 'truck'
    pwm_topic_name = 'pwm_commands'     # Topic that pwm commands are sent over.

    Truck(vehicle_id, address, pwm_topic_name)


if __name__ == '__main__':
    main(sys.argv)