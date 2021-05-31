#!/usr/bin/env python
# license removed for brevity

import rospy

from nav_msgs.msg import OccupancyGrid

from distance_map_class import *


def main():

    dist = distance_map()
    dist.robot_namespace = rospy.get_namespace()
    rospy.init_node('distance_map', anonymous=True)

    ## ROS Subscriber
    rospy.Subscriber("map", OccupancyGrid, dist.localmap_callback, queue_size=2)

    ## ROS Publisher
    dist.pub_distanceMap = rospy.Publisher("map_distance", OccupancyGrid, queue_size=2)    

    rate = rospy.Rate(20)  # 20hz
    while not rospy.is_shutdown():        

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
