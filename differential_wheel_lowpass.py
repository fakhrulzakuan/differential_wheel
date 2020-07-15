#!/usr/bin/env python

""" ARS408 ROS Driver
# Python 2.7
# Author : Fakhrul Zakuan
# Email : razi@moovita.com
# Department : eMooVit, MY
# Last modified: 1-April-2020
# version comments - Almost done
"""

import rospy
import can
import time
from sensor_msgs.msg import Imu
from vi_msgs.msg import WheelSpeed
from pykalman import KalmanFilter
import numpy as np
from scipy.signal import butter,filtfilt, lfilter

import numpy as np
np.set_printoptions(threshold=3)
np.set_printoptions(suppress=True)
from numpy import genfromtxt


class MotionInput:

    def __init__(self):

        rospy.init_node('diff_wheel')

        # topic_wheel_speed = rospy.get_param('~topicSpeed', 'av_base/wheel_speed/ms')
        # topic_yawrate = rospy.get_param('~topicYawRate', 'an_device/Imu')
        
        rospy.Subscriber('av_base/wheelspeed/ms', WheelSpeed, self.SendSpeedInformation)
        self.ang_vel_pub = rospy.Publisher('ang_vel', Imu, queue_size=1)
        self.ang_vel_raw_pub = rospy.Publisher('ang_vel_raw', Imu, queue_size=1)
        self.prev_time = 0.0
        self.prev_ang_vel = 0.0
        self.est_ang_vel = 0.0
        rospy.spin()

        # print "FUCK"
    def SendSpeedInformation(self, data):

        start_time = rospy.Time.now().to_sec()
        delta_t = start_time - self.prev_time


        vr =  data.front_right
        vl =  data.rear_right

        # print vr, vl


        ang_vel = (vr - vl) / 0.63450
        # speed = data.front_right
        # print ang_vel

        av_raw = Imu()
        av_raw.header = data.header
        av_raw.angular_velocity.z = ang_vel

        self.ang_vel_raw_pub.publish(av_raw)
        
        # time.sleep(0.5)
        alpha  = 0.2 #0 - 1
        self.est_ang_vel = alpha * ang_vel + (1-alpha) * self.prev_ang_vel

        self.prev_ang_vel = self.est_ang_vel

        filtered_ang_vel = self.est_ang_vel

        av = Imu()
        av.header = data.header
        av.angular_velocity.z = filtered_ang_vel

        
        self.ang_vel_pub.publish(av)
        print filtered_ang_vel

        self.prev_time = start_time


if __name__ == '__main__':
 
    _MotionInput = MotionInput()
