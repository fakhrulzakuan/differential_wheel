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
        rospy.spin()

        # print "FUCK"
    def SendSpeedInformation(self, data):

        start_time = rospy.Time.now().to_sec()
        delta_t = start_time - self.prev_time
        acceleration = 0
        # print delta_t

        #Transition matrix
        F_t=np.array([ [1 ,0,delta_t,0] , [0,1,0,delta_t] , [0,0,1,0] , [0,0,0,1] ])

        #Initial State cov
        P_t= np.identity(4)*10

        #Process cov
        Q_t= np.identity(4)

        #Control matrix
        B_t=np.array( [ [0] , [0], [0] , [0] ])

        #Control vector
        U_t=acceleration

        #Measurment Matrix
        H_t = np.array([ [1, 0, 0, 0], [ 0, 1, 0, 0]])

        #Measurment cov
        R_t= np.identity(2)*2

        # Initial State
        X_hat_t = np.array( [[0],[0],[0],[0]] )

        # print("X_hat_t",X_hat_t.shape)
        # print("P_t",P_t.shape)
        # print("F_t",F_t.shape)
        # print("B_t",B_t.shape)
        # print("Q_t",Q_t.shape)
        # print("R_t",R_t.shape)
        # print("H_t",H_t.shape)

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

        

        ###########################################################
        #Filter starts here!!

        measurmens = np.array([ang_vel])
        # print measurmens
        X_hat_t,P_hat_t = self.prediction(X_hat_t,P_t,F_t,B_t,U_t,Q_t)
        # print("Prediction:")
        # print("X_hat_t:\n",X_hat_t,"\nP_t:\n",P_t)
        
        Z_t=measurmens.transpose()
        Z_t=Z_t.reshape(Z_t.shape[0],-1)
        
        # print(Z_t.shape)
        
        X_t,P_t=self.update(X_hat_t,P_hat_t,Z_t,R_t,H_t)
        # print("Update:")
        # print("X_t:\n",X_t,"\nP_t:\n",P_t)
        X_hat_t=X_t
        P_hat_t=P_t
        # print X_t
        filtered_ang = X_t[0][0]
        # print("=========================================")
        # print("Opencv Kalman Output:")
        # print("X_t:\n",opencvKalmanOutput[i])

        av = Imu()
        av.header = data.header
        av.angular_velocity.z = filtered_ang

        
        self.ang_vel_pub.publish(av)

        self.prev_time = start_time

    def prediction(self, X_hat_t_1,P_t_1,F_t,B_t,U_t,Q_t):
        X_hat_t=F_t.dot(X_hat_t_1)+(B_t.dot(U_t).reshape(B_t.shape[0],-1) )
        P_t=np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose())))+Q_t

        return X_hat_t,P_t

    def update(self, X_hat_t,P_t,Z_t,R_t,H_t):
        
        K_prime=P_t.dot(H_t.transpose()).dot( np.linalg.inv ( H_t.dot(P_t).dot(H_t.transpose()) +R_t ) )  
        
        X_t=X_hat_t+K_prime.dot(Z_t-H_t.dot(X_hat_t))
        P_t=P_t-K_prime.dot(H_t).dot(P_t)
        
        return X_t,P_t
    


if __name__ == '__main__':
 
    _MotionInput = MotionInput()
