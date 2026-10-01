#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import csv
import os
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import AttitudeTarget


class ThrustCalibration(Node):

    def __init__(self):
        super().__init__('thrust_calibration')

        # Load parameters
        self.time_average_interval = float(
            self.declare_parameter('time_interval', 1.0).value)
        self.vbat_min = float(
            self.declare_parameter('min_battery_voltage', 13.2).value)
        self.mass_kg = float(
            self.declare_parameter('mass_kg', 1.0).value)
        # ROS 1 wrote <px4ctrl source dir>/thrust_calibrate_scrips/data.csv; an installed
        # ROS 2 package has no source dir, so the output file is a parameter.
        self.output_file = os.path.expanduser(str(
            self.declare_parameter('output_file', '~/px4ctrl_thrust_calibrate_data.csv').value))

        # store measurements of vbat and commanded thrusts
        self.volt_buf = np.array([])
        self.volt_records = np.array([])
        self.cmd_buf = np.array([])
        self.cmd_records = np.array([])
        self.last_records_t = self.get_clock().now()

        # Some flags
        self.count_start_trigger_received = 0

        # Subscribe (mavros publishes battery best-effort)
        self.bat_sub = self.create_subscription(
            BatteryState, "/mavros/battery", self.battery_voltage_cb, qos_profile_sensor_data)
        self.create_subscription(
            AttitudeTarget, "/mavros/setpoint_raw/attitude", self.thrust_commands_cb, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/traj_start_trigger", self.start_trigger_cb, 10)

    def battery_voltage_cb(self, msg):

        if (self.count_start_trigger_received == 0):
            return

        self.volt_buf = np.append(
            self.volt_buf, np.array([msg.voltage]), axis=0)

        # Record filted data
        cur_t = self.get_clock().now()
        if ((cur_t - self.last_records_t).nanoseconds * 1e-9 > self.time_average_interval):
            self.last_records_t = cur_t
            self.volt_records = np.append(
                self.volt_records, np.array([np.mean(self.volt_buf)]), axis=0)
            self.volt_buf = np.array([])
            self.cmd_records = np.append(
                self.cmd_records, np.array([np.mean(self.cmd_buf)]), axis=0)
            self.cmd_buf = np.array([])
            print("volt=", self.volt_records[-1], " thr=", self.cmd_records[-1])

        # Calculate calibration parameters
        if ((self.count_start_trigger_received >= 2) or
                (self.volt_records.size != 0 and self.volt_records[-1] < self.vbat_min)):
            self.cal_and_save_data()
            self.destroy_subscription(self.bat_sub)

    def thrust_commands_cb(self, msg):

        if (self.count_start_trigger_received == 0):
            return

        self.cmd_buf = np.append(self.cmd_buf, np.array([msg.thrust]), axis=0)
        print(msg.thrust)

    def start_trigger_cb(self, msg):

        self.count_start_trigger_received += 1
        if(self.count_start_trigger_received == 1):
            self.get_logger().info("Start recording.")
        if(self.count_start_trigger_received > 1):
            self.get_logger().info("Stop recording.")

    def cal_and_save_data(self):
        self.get_logger().info("Data storing.")

        data_stack = np.vstack((self.cmd_records, self.volt_records))
        data = [tuple(x) for x in data_stack.tolist()]

        filedir = self.output_file
        f = open(filedir, 'a')
        writer = csv.writer(f)
        writer.writerow(((time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())),
                         'mass(kg):', self.mass_kg, 'commands', 'voltage'))
        writer.writerows(data)
        f.close()

        self.get_logger().info("Stored to " + filedir)


def main():
    rclpy.init()
    thrust_calibration = ThrustCalibration()
    thrust_calibration.get_logger().info("Waiting for trigger.")
    try:
        rclpy.spin(thrust_calibration)
    except KeyboardInterrupt:
        pass
    thrust_calibration.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
