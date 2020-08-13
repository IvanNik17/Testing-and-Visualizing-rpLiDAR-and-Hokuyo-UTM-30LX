# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 08:43:03 2018

@author: ivan

Testing function for getting easy access to one itteration of lidar data and calculation to carthesian coordinates.
"""

import numpy as np
import math

import matplotlib.pyplot as plt

import UTM_30LX as lx
from helperFunctions_lite import calculateMeans, circle2cart_drone, circle2cart_points

## Subsample lidar data
clustering = 3
##  160 and 880 if we want 0 to 180 degrees  /multiple of 6 
startScanAngle = 0
endScanAngle = 1080

fromDistance = 500
toDistance = 2000



serial_lidar = lx.connect('COM9') 
lx.startLaser(serial_lidar)


angleDistance, status= lx.getLatestScanSteps(serial_lidar, startScanAngle, endScanAngle,clustering)

angleDistance = angleDistance[np.logical_and(angleDistance[:,1] > fromDistance, angleDistance[:,1] < toDistance)]

print(angleDistance)


meanAngle,meanDist = calculateMeans(angleDistance)


dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)




plt.plot(dronePos[0], dronePos[1],30,'ro')

plt.plot(bladePointsPos[:,0],bladePointsPos[:,1],'go')

plt.axes().set_aspect('equal', 'datalim')
    
serial_lidar.close()