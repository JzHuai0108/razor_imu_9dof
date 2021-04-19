#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
To use imu_node to record sparkfun openlog_artemis sensor data,

First upload the firmware with Artemis-Firmware-Upload-GUI.
As of April 1 2021, the firmware version OpenLog_Artemis-V10-v19_BETA in commit 2ce16aa05db1933baf4480c85ff2995632b31872
at git@github.com:sparkfun/OpenLog_Artemis.git has a max logging frequency 230Hz.
But the latest OpenLog_Artemis-V10-v19 in commit 4d833a7f1229c10ca5eb5d78af01843cd7c73f63 and
OpenLog_Artemis-V10-v19-BETA in commit d669a8eda165907538d433f2d542e748f140ad33 have a max logging frequency of 170 Hz.
Then, proceed with the following steps:
1. use tera term or putty connect to it via serial port, configure its baud rate to the maximum value say 500000,
Then in configure terminal output, disable log to microSD, and set the sample rate to 400Hz,
Doing so is because logging to microSD is half as fast as log to a host computer.
Then set accelerometer data range say +/- 4g, gyro range say +/- 500 dps,
also enable accelerometer LPF, and gyro LPF,
also enable microseconds in timestamp configuration.
2. catkin_make razor_imu_9dof
3. source devel/setup.bash
4. rosrun razor_imu_9dof imu_node.py

Depends on pyyaml which can be installed with
sudo pip install pyyaml

Note 1: the upper bound for the device time of microsecond precision is 4295 sec (72 minutes).
So watch out for the time resets.
Note 2: When the cable connects the Raspberry Pi 4B and the openlog artemis,
the Raspberry Pi won't boot up.
Note 3: The program can run in python3 if the ros parts are commented out. Otherwise, python2 is required.

In Ubuntu, use putty to communicate with openlog artemis,
install putty with
sudo apt-get install putty
Then communicate with openlog artemis
sudo putty /dev/ttyUSB0 -serial -sercfg 500000,8,n,1,N
"""

import argparse

import datetime
import warnings

import serial
import signal
import string
import math
import sys
import time

import rospy
from sensor_msgs.msg import Imu
# from diagnostic_msgs.msg import DiagnosticArray

def print_serial_port(ser):
    calib_data = ser.readlines()

    for line in calib_data:
        print(line)

class ImuRecorder(object):
    def __init__(self):
        self.pub = None
        self.imuMsg = None
        self.seq = 0
        self.queue_size = 1
        self.serialPort = None
        self.logstream = None
        self.deviceRefDate = None

    def initRosNode(self):
        rospy.init_node("imu_node")
        # We only care about the most recent measurement, i.e. queue_size=1
        self.pub = rospy.Publisher('/imu0', Imu, queue_size = self.queue_size)
        # diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
        # diag_pub_time = rospy.get_time()

        self.imuMsg = Imu()

        # Orientation covariance estimation:
        # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
        # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
        # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
        # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
        # i.e. variance in yaw: 0.0025
        # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
        # static roll/pitch error of 0.8%, owing to gravity orientation sensing
        # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
        # so set all covariances the same.
        self.imuMsg.orientation_covariance = [
            0.0025 , 0 , 0,
            0, 0.0025, 0,
            0, 0, 0.0025 ]

        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        self.imuMsg.angular_velocity_covariance = [
            0.02, 0 , 0,
            0 , 0.02, 0,
            0 , 0 , 0.02 ]

        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        self.imuMsg.linear_acceleration_covariance = [
            0.04 , 0 , 0,
            0 , 0.04, 0,
            0 , 0 , 0.04 ]

    def openSerialPort(self, port, baudrate):
        print("Opening {}...".format(port))
        try:
            self.serialPort = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        except serial.serialutil.SerialException:
            warnings.warn("IMU not found at port "+ port + ". Did you specify the correct port in the launch file?\n"
                        "Go to /dev/ttyUSB* to check the USB port number. If need be, 'sudo chmod 777 /dev/ttyUSB0'.")
            sys.exit(0)

    def openLogStream(self, output_txt):
        self.logstream = open(output_txt, 'w')
        self.logstream.write('#host-timestamp[sec],gx(rad/s),gy,gz,ax(m/s^2),ay,az,device-time[sec],date-time[sec],temperature,rate\n')

    def closeLogStream(self):
        self.serialPort.close
        self.logstream.close

    def flushSerialPort(self, hostBaselineTime):
        print("Giving the razor IMU board a few seconds to boot...")
        time.sleep(2)
        cmd = 'h' + chr(13)
        self.serialPort.write(cmd.encode())
        time.sleep(1)
        print_serial_port(self.serialPort)

        cmd = 'x' + chr(13)
        self.serialPort.write(cmd.encode())
        time.sleep(1)

        print("Flushing first few IMU entries...")

        while True:
            binaryline = self.serialPort.readline()
            # line = binaryline.decode('ascii')  # decode is needed for python3.
            line = binaryline
            words = str.split(line, ",")
            if len(words) > 2:
                rtcDate = words[0]
                m, d, y = rtcDate.split('/')
                self.deviceRefDate = datetime.datetime(int(y), int(m), int(d))
                self.logstream.write('#Time to start recording in host clock {} Device reference date {}\n'.
                                format(hostBaselineTime, self.deviceRefDate))
                print('Device reference date {}'.format(self.deviceRefDate))
                break

    def publishImu(self, deviceTime, axyz, gxyz):
        self.imuMsg.header.stamp = rospy.Time.from_sec(deviceTime)
        self.imuMsg.header.frame_id = 'base_imu_link'
        self.imuMsg.header.seq = self.seq
        self.imuMsg.linear_acceleration.x = axyz[0]
        self.imuMsg.linear_acceleration.y = axyz[1]
        self.imuMsg.linear_acceleration.z = axyz[2]
        self.imuMsg.angular_velocity.x = gxyz[0]
        self.imuMsg.angular_velocity.y = gxyz[1]
        self.imuMsg.angular_velocity.z = gxyz[2]
        self.seq = self.seq + 1
        self.pub.publish(self.imuMsg)

    def logImuLoop(self):
        print("Publishing IMU data...")
        while True:
            try:
                binaryline = self.serialPort.readline()
                # line = binaryline.decode('ascii')  # decode is needed for python3.
                line = binaryline
                words = str.split(line, ",")
                # date, time, accel, gyro, magnetometer, temperature, rate
                # example words: ['01/01/2000', '00:04:04.34', '128238929', '-1.95', '491.70', '-854.98',
                # '2.02', '-0.11', '-0.44', '-38.55', '51.45', '-129.90', '31.05', '85.01', '\r\n']

                if len(words) <= 2:
                    continue
                # see https://github.com/sparkfun/OpenLog_Artemis/blob/master/SENSOR_UNITS.md
                accel_factor = 9.80665 / 1000.0    # sensor reports accel in units of 1 milli G (9.8m/s^2). Convert to m/s^2.
                gyro_factor = math.pi / 180
                accel_start_index = 3
                axyz = [float(words[accel_start_index]) * accel_factor,
                        float(words[accel_start_index + 1]) * accel_factor,
                        float(words[accel_start_index + 2]) * accel_factor]
                gxyz = [float(words[accel_start_index + 3]) * gyro_factor,
                        float(words[accel_start_index + 4]) * gyro_factor,
                        float(words[accel_start_index + 5]) * gyro_factor]

                rtcDate = words[0]
                rtcTime = words[1]
                rtcSecs = float(words[2]) / 1000000
                mon, d, y = rtcDate.split('/')
                h, minute, s = rtcTime.split(':')
                floatSec = float(s)
                integerSec = int(floatSec)
                decimalMicrosec = int((floatSec - integerSec) * 1000000)
                dateTime = datetime.datetime(int(y), int(mon), int(d), int(h), int(minute), integerSec, decimalMicrosec)
                elapsedDeviceTime = dateTime - self.deviceRefDate
                elapsedSecs = elapsedDeviceTime.total_seconds()

                temperature = words[-3]
                rate = words[-2]
                currentTime = time.time()
                message = "{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.2f},{},{}".format(
                    currentTime, gxyz[0], gxyz[1], gxyz[2], axyz[0], axyz[1], axyz[2],
                    rtcSecs, elapsedSecs, temperature, rate)
                self.logstream.write("{}\n".format(message))

                self.publishImu(rtcSecs, axyz, gxyz)

            except Exception as e:
                print(e)


def main():
    parser = argparse.ArgumentParser(description='Get parameters for imu_node.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--baudrate', metavar='baudrate', type=int, default=500000,
                        help='baudrate to connect to the serial port of the sparkfun IMU')
    parser.add_argument('--output_txt', metavar='output_txt', type=str, default='',
                        help='output txt')
    parser.add_argument('--port', metavar='port', type=str, default='/dev/ttyUSB0',
                        help='IMU USB port')
    args = parser.parse_args()
    hostBaselineTime = None
    if not args.output_txt:
        hostBaselineTime = datetime.datetime.now()
        timestr = hostBaselineTime.strftime("%Y%m%d-%H%M%S")
        args.output_txt = '{}.log'.format(timestr)

    recorder = ImuRecorder()
    recorder.openSerialPort(args.port, int(args.baudrate))
    recorder.initRosNode()
    recorder.openLogStream(args.output_txt)

    # https://stackoverflow.com/questions/12371361/using-variables-in-signal-handler-require-global
    def signal_handler(sig, frame):
        print('Closing serial port and log stream...!')
        recorder.closeLogStream()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    recorder.flushSerialPort(hostBaselineTime)
    print("Start data stream to {}...".format(args.output_txt))
    recorder.logImuLoop()

if __name__ == "__main__":
    main()
