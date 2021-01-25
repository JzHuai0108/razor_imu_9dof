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
1. use tera term or putty connect to it via serial port, configure its baud rate to the maximum value say 500000,
Then configure the sample rate to 400Hz,
Then set accelerometer data range say +/- 4g, gyro range say +/- 500 dps, also enable accelerometer LPF, and gyro LPF.
2. catkin_make razor_imu_9dof
3. source devel/setup.bash
4. rosrun razor_imu_9dof imu_node.py
"""

import argparse
import rospy
import serial
import string
import math
import sys

from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray

def print_serial_port(ser):
    calib_data = ser.readlines()
    calib_data_print = ""
    for line in calib_data:
        calib_data_print += line
    rospy.loginfo(calib_data_print)

def main():
    parser = argparse.ArgumentParser(description='Get parameters for imu_node.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--baudrate', metavar='baudrate', type=int, default=500000,
                        help='baudrate to connect to the serial port of the sparkfun IMU')
    parser.add_argument('--output_txt', metavar='output_txt', type=str, default='',
                        help='output txt')
    args = parser.parse_args()
    if not args.output_txt:
        import time
        timestr = time.strftime("%Y%m%d-%H%M%S")
        args.output_txt = '{}.log'.format(timestr)

    baudrate = int(args.baudrate)

    rospy.init_node("imu_node")
    #We only care about the most recent measurement, i.e. queue_size=1
    pub = rospy.Publisher('imu', Imu, queue_size=1)
    diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
    diag_pub_time = rospy.get_time()
    rospy.loginfo("Output file {}".format(args.output_txt))

    imuMsg = Imu()

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
    imuMsg.orientation_covariance = [
    0.0025 , 0 , 0,
    0, 0.0025, 0,
    0, 0, 0.0025
    ]

    # Angular velocity covariance estimation:
    # Observed gyro noise: 4 counts => 0.28 degrees/sec
    # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
    # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
    imuMsg.angular_velocity_covariance = [
    0.02, 0 , 0,
    0 , 0.02, 0,
    0 , 0 , 0.02
    ]

    # linear acceleration covariance estimation:
    # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
    # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
    # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
    imuMsg.linear_acceleration_covariance = [
    0.04 , 0 , 0,
    0 , 0.04, 0,
    0 , 0 , 0.04
    ]

    default_port='/dev/ttyUSB0'
    port = rospy.get_param('~port', default_port)

    rospy.loginfo("Opening %s...", port)
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    except serial.serialutil.SerialException:
        rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?\n"
                     "Go to /dev/ttyUSB* to check the USB port number. If need be, 'sudo chmod 777 /dev/ttyUSB0'.")
        sys.exit(0)

    seq = 0
    # see https://github.com/sparkfun/OpenLog_Artemis/blob/master/SENSOR_UNITS.md
    accel_factor = 9.80665 / 1000.0    # sensor reports accel in units of 1 milli G (9.8m/s^2). Convert to m/s^2.
    gyro_factor = math.pi / 180
    rospy.loginfo("Giving the razor IMU board a few seconds to boot...")
    rospy.sleep(2)

    ser.write('h' + chr(13))
    rospy.sleep(1)
    print_serial_port(ser)

    rospy.loginfo("Start data stream")
    ser.write('x' + chr(13))
    rospy.sleep(1)

    rospy.loginfo("Flushing first few IMU entries...")
    for x in range(0, 20):
        ser.readline()

    rospy.loginfo("Publishing IMU data...")
    f = open(args.output_txt, 'w')
    f.write('timestamp[sec],gx(rad/s),gy,gz,ax(m/s^2),ay,az,date,time,temperature,rate\n');
    while not rospy.is_shutdown():
        line = ser.readline()

        words = string.split(line, ",")
        # date, time, accel, gyro, magnetometer, temperature, rate
        # example words: ['01/01/2000', '00:04:04.34', '-1.95', '491.70', '-854.98',
        # '2.02', '-0.11', '-0.44', '-38.55', '51.45', '-129.90', '31.05', '85.01', '\r\n']

        if len(words) > 2:
            accel_start_index = 2
            imuMsg.linear_acceleration.x = float(words[accel_start_index]) * accel_factor
            imuMsg.linear_acceleration.y = float(words[accel_start_index + 1]) * accel_factor
            imuMsg.linear_acceleration.z = float(words[accel_start_index + 2]) * accel_factor

            imuMsg.angular_velocity.x = float(words[accel_start_index + 3]) * gyro_factor
            imuMsg.angular_velocity.y = float(words[accel_start_index + 4]) * gyro_factor
            imuMsg.angular_velocity.z = float(words[accel_start_index + 5]) * gyro_factor

            rtc_date = words[0]
            rtc_time = words[1]
            temperature = words[-3]
            rate = words[-2]

            imuMsg.header.stamp= rospy.Time.now()
            imuMsg.header.frame_id = 'base_imu_link'
            imuMsg.header.seq = seq
            seq = seq + 1
            pub.publish(imuMsg)
            message = "{}.{:09d},{},{},{},{},{},{},{},{},{},{}".format(
                imuMsg.header.stamp.secs, imuMsg.header.stamp.nsecs,
                imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z,
                imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z,
                rtc_date, rtc_time, temperature, rate)
            f.write("{}\n".format(message))
            # print(message)

    ser.close
    f.close


if __name__ == "__main__":
    main()
