#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import ina228_smbus
import ina_msgs.msg

class INA228_Pub():

    def __init__(self):
        # Parameter
        bus_number = rospy.get_param('~bus_number', 1)
        address = rospy.get_param('~address', 0x40)
        shunt = rospy.get_param('~shunt', 0.002)
        max_current = rospy.get_param('~max_current', 70)
        self.publish_rate = rospy.get_param('~publish_rate', 2)

        # Device setup
        self.ina = ina228_smbus.INA228_SMBus(bus_number, address)
        self.ina.setShunt(shunt, max_current)
        self.ina.averaging_count = ina228_smbus.AveragingCount.COUNT_16

        # Publisher
        self.pub = rospy.Publisher("ina228", ina_msgs.msg.INA228, queue_size=10)

    def spin(self):
        msg = ina_msgs.msg.INA228()
        r = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            #print(self.ina.current_A)

            msg.header.stamp = rospy.Time.now()
            msg.current = self.ina.current_A
            msg.voltage = self.ina.voltage_V
            msg.temp = self.ina.temperature_degc
            msg.power = self.ina.power_W
            msg.energy = self.ina.energy_J
            msg.charge = self.ina.charge_C
            self.pub.publish(msg)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('ina228')
    ina228ros = INA228_Pub()
    ina228ros.spin()
