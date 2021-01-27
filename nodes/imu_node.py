#!/usr/bin/env python3

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
1. use tera term or putty connect to it via serial port, configure its baud rate to the maximum value say 500000,
Then in configure terminal output, disable log to microSD, and set the sample rate to 400Hz,
Doing so is because logging to microSD is half as fast as log to a host computer.
Then set accelerometer data range say +/- 4g, gyro range say +/- 500 dps,
also enable accelerometer LPF, and gyro LPF,
also enable microseconds in timestamp configuration.
2. catkin_make razor_imu_9dof
3. source devel/setup.bash
4. rosrun razor_imu_9dof imu_node.py

Note 1: the upper bound for the device time of microsecond precision is 4295 sec (72 minutes).
So watch out for the time resets.
Note 2: When the cable connects the Raspberry Pi 4B and the openlog artemis,
the Raspberry Pi won't boot up.

In Ubuntu, use putty to communicate with openlog artemis,
install putty with
sudo apt-get install putty
Then communicate with openlog artemis
sudo putty /dev/ttyUSB0 -serial -sercfg 500000,8,n,1,N
"""

import argparse
# import rospy
import datetime
import warnings

import serial
import signal
import string
import math
import sys
import time

# from sensor_msgs.msg import Imu
# from diagnostic_msgs.msg import DiagnosticArray

def print_serial_port(ser):
    calib_data = ser.readlines()

    for line in calib_data:
        print(line)


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

    baudrate = int(args.baudrate)

    # rospy.init_node("imu_node")
    #We only care about the most recent measurement, i.e. queue_size=1
    # pub = rospy.Publisher('imu', Imu, queue_size=1)
    # diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
    # diag_pub_time = rospy.get_time()

    # imuMsg = Imu()

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
    # imuMsg.orientation_covariance = [
    # 0.0025 , 0 , 0,
    # 0, 0.0025, 0,
    # 0, 0, 0.0025 ]

    # Angular velocity covariance estimation:
    # Observed gyro noise: 4 counts => 0.28 degrees/sec
    # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
    # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
    # imuMsg.angular_velocity_covariance = [
    # 0.02, 0 , 0,
    # 0 , 0.02, 0,
    # 0 , 0 , 0.02 ]

    # linear acceleration covariance estimation:
    # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
    # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
    # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
    # imuMsg.linear_acceleration_covariance = [
    # 0.04 , 0 , 0,
    # 0 , 0.04, 0,
    # 0 , 0 , 0.04 ]

    # arg.port = rospy.get_param('~port', args.port)

    print("Opening {}...".format(args.port))
    try:
        serialPort = serial.Serial(port=args.port, baudrate=baudrate, timeout=1)
    except serial.serialutil.SerialException:
        warnings.warn("IMU not found at port "+args.port + ". Did you specify the correct port in the launch file?\n"
                      "Go to /dev/ttyUSB* to check the USB port number. If need be, 'sudo chmod 777 /dev/ttyUSB0'.")
        sys.exit(0)

    logstream = open(args.output_txt, 'w')
    logstream.write('#host-timestamp[sec],gx(rad/s),gy,gz,ax(m/s^2),ay,az,device-time[sec],date-time[sec],temperature,rate\n')

    # https://stackoverflow.com/questions/12371361/using-variables-in-signal-handler-require-global
    def signal_handler(sig, frame):
        print('Closing serial port and log stream...!')
        serialPort.close
        logstream.close
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # seq = 0
    # see https://github.com/sparkfun/OpenLog_Artemis/blob/master/SENSOR_UNITS.md
    accel_factor = 9.80665 / 1000.0    # sensor reports accel in units of 1 milli G (9.8m/s^2). Convert to m/s^2.
    gyro_factor = math.pi / 180
    print("Giving the razor IMU board a few seconds to boot...")
    time.sleep(2)
    cmd = 'h' + chr(13)
    serialPort.write(cmd.encode())
    time.sleep(1)
    print_serial_port(serialPort)

    print("Start data stream to {}...".format(args.output_txt))
    cmd = 'x' + chr(13)
    serialPort.write(cmd.encode())
    time.sleep(1)

    print("Flushing first few IMU entries...")
    deviceRefDate = None
    while True:
        binaryline = serialPort.readline()
        line = binaryline.decode('ascii')
        words = str.split(line, ",")
        if len(words) > 2:
            rtcDate = words[0]
            m, d, y = rtcDate.split('/')
            deviceRefDate = datetime.datetime(int(y), int(m), int(d))
            logstream.write('#Time to start recording in host clock {} Device reference date {}\n'.
                            format(hostBaselineTime, deviceRefDate))
            print('Device reference date {}'.format(deviceRefDate))
            break

    print("Publishing IMU data...")
    while True:
        try:
            binaryline = serialPort.readline()
            line = binaryline.decode('ascii')
            words = str.split(line, ",")
            # date, time, accel, gyro, magnetometer, temperature, rate
            # example words: ['01/01/2000', '00:04:04.34', '128238929', '-1.95', '491.70', '-854.98',
            # '2.02', '-0.11', '-0.44', '-38.55', '51.45', '-129.90', '31.05', '85.01', '\r\n']

            if len(words) <= 2:
                continue

            accel_start_index = 3
            axyz = [float(words[accel_start_index]) * accel_factor,
                    float(words[accel_start_index + 1]) * accel_factor,
                    float(words[accel_start_index + 2]) * accel_factor]
            gxyz = [float(words[accel_start_index + 3]) * gyro_factor,
                    float(words[accel_start_index + 4]) * gyro_factor,
                    float(words[accel_start_index + 5]) * gyro_factor]

            # imuMsg.header.stamp = rospy.Time.now()
            # imuMsg.header.frame_id = 'base_imu_link'
            # imuMsg.header.seq = seq
            # imuMsg.linear_acceleration.x = axyz[0]
            # imuMsg.linear_acceleration.y = axyz[1]
            # imuMsg.linear_acceleration.z = axyz[2]
            # imuMsg.angular_velocity.x = gxyz[0]
            # imuMsg.angular_velocity.y = gxyz[1]
            # imuMsg.angular_velocity.z = gxyz[2]
            # seq = seq + 1
            # pub.publish(imuMsg)

            rtcDate = words[0]
            rtcTime = words[1]
            rtcSecs = float(words[2]) / 1000000
            mon, d, y = rtcDate.split('/')
            h, minute, s = rtcTime.split(':')
            floatSec = float(s)
            integerSec = int(floatSec)
            decimalMicrosec = int((floatSec - integerSec) * 1000000)
            dateTime = datetime.datetime(int(y), int(mon), int(d), int(h), int(minute), integerSec, decimalMicrosec)
            elapsedDeviceTime = dateTime - deviceRefDate
            elapsedSecs = elapsedDeviceTime.total_seconds()

            temperature = words[-3]
            rate = words[-2]
            currentTime = time.time()
            message = "{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.2f},{},{}".format(
                currentTime, axyz[0], axyz[1], axyz[2], gxyz[0], gxyz[1], gxyz[2],
                rtcSecs, elapsedSecs, temperature, rate)
            logstream.write("{}\n".format(message))
            # print(message)
        except Exception as e:
            print(e)


if __name__ == "__main__":
    main()
