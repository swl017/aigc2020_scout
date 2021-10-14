#!/usr/bin/env python
# license removed for brevity
from __future__ import division

import numpy as np
import math
import tf

from nav_msgs.msg import OccupancyGrid
from scipy import ndimage


class distance_map:
    def __init__(self):
        self.robot_namespace = 'c'
            
        self.normalizedDistMap = [0][0]
        self.OriginalMap = [0][0]      
        self.localmap = OccupancyGrid()
        self.distMap = OccupancyGrid()       
        self.pub_distanceMap = 0
        self.OccupancyValue = 0
        self.MapInfo_resolution = 0.0
        self.MapInfo_available = 0

        self.count = 0.0
        self.t_cur = 0.0


    def localmap_callback(self, data):
        self.localmap.header = data.header
        self.localmap.info = data.info
        self.localmap.data = data.data
        self.MapInfo_resolution = self.localmap.info.resolution
        self.MapInfo_available = 1
        if self.MapInfo_available == 1:
                self.genDistanceMap()


    def genDistanceMap(self):

        self.distMap.header.stamp = self.localmap.header.stamp
        self.distMap.header.frame_id = self.robot_namespace + "map"
        self.distMap.info.width = self.localmap.info.width
        self.distMap.info.height = self.localmap.info.height
        self.distMap.info.origin.orientation.w = 1.0
        self.distMap.info.resolution = self.MapInfo_resolution
        self.distMap.info.origin.position.x = self.localmap.info.origin.position.x
        self.distMap.info.origin.position.y = self.localmap.info.origin.position.y

        MaxX = self.localmap.info.width
        MaxY = self.localmap.info.height

        self.OriginalMap = np.zeros(shape=(MaxY, MaxX))
        distanceMap = np.zeros(shape=(MaxY, MaxX))

        maxd = 0.0
        num = 0
        num2 = 0
        num3 = 0
        num4 = 0
               
        self.OccupancyValue = 0
        for num in range(MaxY):
            for num2 in range(MaxX):               
                if self.localmap.data[num2+(MaxX)*num]==-1:
                    self.OccupancyValue = 0              
                elif self.localmap.data[num2+(MaxX)*num]>50:
                    self.OccupancyValue = 0
                else:
                    self.OccupancyValue = 1

                self.OriginalMap[num][num2] = self.OccupancyValue

                if ( num >=  MaxY ) or ( num2 >=  MaxX ):
                    self.OriginalMap[num][num2] = 0   

        distanceMap = ndimage.distance_transform_edt(self.OriginalMap)        
        maxd = (max(map(max,distanceMap)))
        distanceMap = (distanceMap/maxd*100)
        self.normalizedDistMap = distanceMap.astype(int)  

        for num3 in range(MaxY):
            for num4 in range(MaxX):
                self.distMap.data.append(self.normalizedDistMap[num3][num4])      

        self.pub_distanceMap.publish(self.distMap)
        self.distMap.data=[]
