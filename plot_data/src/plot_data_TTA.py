#!/usr/bin/env python
# license removed for brevity

import numpy as np
import cv2
import sys

import rospy
import tf

import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Float32MultiArray, Header, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, Pose, Point, Vector3


def main():

    rospy.init_node('plot_datalog', anonymous=True)
    #filename = "/home/usrg/"+data+".txt"
    filename = "/home/nvidia/data1.txt"
    fd = open(filename, 'r')
    data = fd.read()
    fd.close()

    time = []
    pos_x = []
    pos_y = []
    pos_z = []
    opti_x = []
    opti_y = []
    opti_z = []

    count_line = 0

    for line in data.split('\n'):
        var = line.split()
        if count_line == 1500:
            break
        if count_line > 2:
            if not var:
                break
            time.append(float(var[0]))
            pos_x.append(float(var[1]))
            pos_y.append(float(var[2]))
            pos_z.append(float(var[3]))

        count_line = count_line + 1

    filename = "/home/nvidia/data1.csv"
    fd = open(filename, 'r')
    data = fd.read()
    fd.close()

    opti_t = []
    opti_x = []
    opti_y = []
    opti_z = []

    count_line = 0

    for line in data.split('\r\n'):
        var = line.split(',')
        if count_line == 1500:
            break
        if count_line > -1:
            if not var:
                break
            opti_t.append(float(var[1]))
            opti_x.append(float(var[8]))
            opti_y.append(float(var[6]))
            opti_z.append(float(var[7]))

        count_line = count_line + 1

    print("plot on!")

    err_x = []
    err_y = []

    sum_x_2 = 0
    sum_y_2 = 0

    zip_object = zip(opti_x, pos_x)
    for opti_i, pos_i in zip_object:
        err_x.append(opti_i - pos_i)
        sum_x_2 = sum_x_2 + (opti_i - pos_i)*(opti_i - pos_i)

    zip_object = zip(opti_y, pos_y)
    for opti_i, pos_i in zip_object:
        err_y.append(opti_i - pos_i)
        sum_y_2 = sum_y_2 + (opti_i - pos_i)*(opti_i - pos_i)

    RMSE_x = np.sqrt(sum_x_2 / 1495.0)
    RMSE_y = np.sqrt(sum_y_2 / 1495.0)

    print("opti time : %.3f  slam time : %.3f" %(opti_t[0], time[0]))
    print("RMSE : [x] %.3f[m]  [y] %.3f[m]" % (RMSE_x, RMSE_y))


    
    fig = plt.figure(1)
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot(time, pos_x, opti_t, opti_x)
    plt.ylabel('pos x')
    plt.title('position data')
    plt.grid(True)

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(time, pos_y, opti_t, opti_y)
    plt.xlabel('sec')
    plt.ylabel('pos y')
    plt.grid(True)

    
    fig = plt.figure(2)
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot(time, err_x)
    plt.ylabel('err_x')
    plt.title('error data')
    plt.grid(True)

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(time, err_y)
    plt.ylabel('err_y')
    plt.grid(True)
    '''
    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(time, pos_z, time, pos_tf_z, time, pos_qsf_z, time, hover_z)
    plt.ylabel('pos_z')
    plt.grid(True)

    
    ax3 = fig.add_subplot(4, 1, 4)
    ax3.plot(time, mission)
    plt.ylabel('mission')
    plt.xlabel('sec')
    plt.grid(True)
    '''

    plt.show()
    rate = rospy.Rate(2)  # 20hz
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
