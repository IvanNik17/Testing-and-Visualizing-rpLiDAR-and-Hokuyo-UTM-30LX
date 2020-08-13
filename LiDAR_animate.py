# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 09:04:56 2018

@author: ivan

Visualizer for the Lidar's data. It can used with both Hokuyo and the rpLidar. It contains the core/bare minimum functions to get the data
transform it, calculate necessary angles and orientations, correct the data and display it.

To be fully working the script needs both a lidar and a IMU/Arduino combo, but with small changes 
"""

# Default imports
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial


# LiDAR driver/library import - UTM-30LX lidar and rpLiDAR 
import UTM_30LX as lx

from rplidar import RPLidar
from rplidar import RPLidarException

# Imports from the helper library - lite version that does not contains only the necessary minimum. They are explained in the library itself
from helperFunctions_lite import calculateMeans, circle2cart_drone, circle2cart_points, calculateAnglesPCA, testEllipse, calculateMinMax, intersectionLineCurve,getDataFromIMU

#Class for animating the visualization
class AnimatedScatter(object):
    """An animated scatter plot using matplotlib.animations.FuncAnimation. WhichLidar -> 0 = rplidar, 1 = NONE, 2 = hokuyo
        MaxRange, AxisRange - ranges only for visualization
        lidarMinThresh, lidarMaxThresh - distance thresholds for what data is taken from the lidar reading
        whichCenter - only for demonstration purposes - if 0 then lidar is center of coordinate system, if 1 then the lidar's position is calculated from all it's readings to a arbitrary 0 based coordinate system
    """
    
    def __init__(self,portName = '/dev/ttyACM0',  maxRange = 30000, axisRange = 10000, whichLidar = 2, lidarMinThresh = 500, lidarMaxThresh = 5000, whichCenter = 0):
        
        #===============================================================#
        #=================== Initialization Block ======================#
        #===============================================================#
        
        self.maxRange = maxRange
        self.axisRange = axisRange
        
        self.whichLidar = whichLidar
        self.portName = portName
        
        self.lidarMinThresh = lidarMinThresh
        self.lidarMaxThresh = lidarMaxThresh
        
        self.whichCenter = whichCenter
        
        # global variables       
        self.anglePCA = 0 # angle of the blade points, calculated using PCA
        self.PCACalculated = False # helper bool for determining if blade angle calculated
        
        self.environmentPoints = [] # detected points from blade

        self.calculateEllipse = False # is the ellipse calculated
        
        self.ellipseAlgStart = False # is the elliptical algorithm running

        
        self.serial_IMU = serial.Serial("COM10",57600) # IMU + arduino serial, if no IMU is present please comment out
        
        self.arduinoInitial = np.zeros(8)#if no IMU is present please comment out
        self.lidarRotAngle = 0 # lidar rotation angle , if no IMU is present please comment out
        
        self.armedAngle = 0 # lidar angle when the blade angle is calculated, used as an initial angle, if no IMU is present please comment out
        
        # initialize lidar - in case of the rpLidar  an initial health check is required to be sure that the proper data is sent     
        if self.whichLidar is 0:
        
            self.lidar = RPLidar(self.portName)
            
            print("Starting Lidar...")
            ## Start Lidar and get's info
            while True:
                try:
                    
                    info = self.lidar.get_health()
                    
                    
                    break
                except RPLidarException:
                    
                    print("Lidar error retry")
                finally:
                    self.lidar.stop()
            
            print(info)
            
            self.iterator = self.lidar.iter_scans(1000,5,100,6000)
        
        elif self.whichLidar is 1:
            pass
            # The sweep lidar is removed as it is not currently used 
            
        
        elif self.whichLidar is 2:

            self.lidar = lx.connect(self.portName) 
            
            lx.startLaser(self.lidar)

        #===============================================================#
        #========================== BLOCK END ==========================#
        #===============================================================#
        
        
        #===============================================================#
        #================== Setup figures and events ===================#
        #===============================================================#
        
        # Setup the figure and axes...
        self.fig, self.ax = plt.subplots()
        
        self.fig.canvas.mpl_connect('close_event', self.handle_close) # event for clicking the X
        
        self.onClickEv = self.fig.canvas.mpl_connect('button_press_event', self.on_click) # event for clicking the mouse button
        # Then setup FuncAnimation. - self.update is the update function, while self.setup_plot is the initialization of figures
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=1./40, 
                                           init_func=self.setup_plot, blit=True)
        
        
        #===============================================================#
        #========================== BLOCK END ==========================#
        #===============================================================#


    # Handler function for closing the figure, each lidar has different exit strategies   
    def handle_close(self,evt):
        print('Closed Figure!')
        
        self.fig.canvas.mpl_disconnect(self.onClickEv) 
        self.serial_IMU.close() # disconnect IMU serial, if no IMU is present please comment out
        if self.whichLidar is 0:
            
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        elif self.whichLidar is 1:
            pass
        
        elif self.whichLidar is 2:
            self.lidar.close()
            
    # Handler function for mouse click on the canvas         
    def on_click(self, evt):
        
        # if the blade points orientation is calculated and the elliptical alg is not started - start it now       
        if self.PCACalculated == True and self.ellipseAlgStart == False:
            self.ellipseAlgStart = True
        
        # if the blade points orientation is not calculated - do it       
        if self.PCACalculated == False:
            
            self.anglePCA = calculateAnglesPCA(self.environmentPoints)
            self.PCACalculated = True
            
            print(self.anglePCA)
            
        
        print("Clicked")
        
    # Setup function for plotting       
    def setup_plot(self):

        """Setup static markings."""

        # Setup axis limits
        self.ax.axis([-self.axisRange, self.axisRange, -self.axisRange, self.axisRange])

        # Create updating scatter plot and variable for later use
        self.scat = self.ax.scatter([], [], c='b', s=1, animated=True, alpha=0.5)
        
        self.scat_lidar = self.ax.scatter([], [], c='r', s=30, animated=True, alpha=0.5)
        
        self.scat_ellipse = self.ax.scatter([], [], c='g', s=20, animated=True, alpha=0.5)

        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,self.scat_lidar,self.scat_ellipse,

    def update(self, i):
        """Update the scatter plot."""
        
        # Update function runs and gets data from the Lidar       
        if self.whichLidar is 0:
          scan = next(self.iterator)
          angleDistance = np.array(scan)  
            
        elif self.whichLidar is 1:
            pass
        
        elif self.whichLidar is 2:
            # 0 and 1080 is the degrees that the Hokuyo gets readings from - 0 to 180 degrees, as it checks in 6 increments between a degree
            # the last value of 1 signifies if the data will be clustered - if it's 1 data is not averaged by captures           
            angleDistance, status = lx.getLatestScanSteps(self.lidar, 0, 1080,1)
        
        # Remove data that is farther or closer than the thresholds        
        angleDistance = angleDistance[np.logical_and(angleDistance[:,1] > self.lidarMinThresh, angleDistance[:,1] < self.lidarMaxThresh)]
        
        # Used only for testing/visualization purposes - when 0 only the environment points are printed and the lidar is always at 0,0        
        if self.whichCenter == 0:
            
            environmentPoints = circle2cart_points([0,0],angleDistance, 0)
            lidarPoint = [0,0]
        # Start real algorithm        
        elif self.whichCenter == 1:
            
            # Get orientation data from IMU, if no IMU present comment out the next three lines             
            arduinoOutput = getDataFromIMU(self.serial_IMU,self.arduinoInitial)
            self.arduinoInitial = arduinoOutput
            self.lidarRotAngle = arduinoOutput[4]

            # if the elliptical algorithm is started compensate for the lidar's rotation            
            if self.ellipseAlgStart is True:

                compensateYaw = self.lidarRotAngle - self.armedAngle # current rotation angles - the armed angle
        
                angleDistance[:,0] = angleDistance[:,0] + compensateYaw 
            
            # Calculate mean angle and mean distance          
            meanAngle,meanDist = calculateMeans(angleDistance)
            
            # go from polar to carthesian system - position the lidar at a 0,0 coordinate system, then reproject blade points from the lidar's position            
            lidarPoint = circle2cart_drone([0,0],meanAngle, meanDist)
            environmentPoints = circle2cart_points(lidarPoint,angleDistance, 0)
            
            self.environmentPoints = environmentPoints
            
            # is the blade orientation calculated
            if self.PCACalculated is True:
                
                
                if self.calculateEllipse is False:
                    # Calculate ellipse - first get the major diameter of the ellipse
                    minDist,maxDist,minDist_points,maxDist_points = calculateMinMax(self.environmentPoints)
                    
                    # create ellipse using the major diameter, and major diameter/6 as minor diameter, the angle of rotation and a 0,0 center
                    ellipseRadAngle, self.ellipseBladePoints = testEllipse(int(maxDist/6), int(maxDist), self.anglePCA, [0,0])
                    self.calculateEllipse = True
                    
                    # save the armed angle of lidar orientation
                    self.armedAngle = self.lidarRotAngle
                    
                if self.ellipseAlgStart is True:
                    
                    # get the average detected point position
                    averagePointPos=[sum(environmentPoints[:,0])/len(environmentPoints[:,0]),sum(environmentPoints[:,1])/len(environmentPoints[:,1])]
                    pCenter=np.array((0,0))
                    pAvg = np.array((averagePointPos[0],averagePointPos[1]))
                    # calculate the distance between the ellipse center and the point
                    distAveragePointToZero = np.linalg.norm(pCenter-pAvg)
                    
                    # find the intersection between the ellipse the center of the ellipse and the lidar position
                    intersectPointOnCurve = intersectionLineCurve([0,0], lidarPoint, self.ellipseBladePoints)
                    
                    # calculate correction distance - ellipse radius 
                    correctionDist = math.sqrt(intersectPointOnCurve[0]**2 + intersectPointOnCurve[1]**2)
                    # calculate new center for going from polar to carthesian using the correction distance and the dist of the average point to zero
                    newCenterPos = circle2cart_drone([0,0],meanAngle, correctionDist -distAveragePointToZero)
                    
                    lidarPoint = circle2cart_drone(newCenterPos,meanAngle, meanDist)
                    
                    environmentPoints = circle2cart_points(lidarPoint,angleDistance, 0)
                # visualize ellipse
                self.scat_ellipse.set_offsets(self.ellipseBladePoints)


        # Visualize environment and lidar points 
        self.scat.set_offsets(environmentPoints)
        
        self.scat_lidar.set_offsets(lidarPoint)

        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,self.scat_lidar,self.scat_ellipse,

    def show(self):
        plt.show()
        



if __name__ == '__main__':

        
    a = AnimatedScatter("COM9",1000,1000,2, 100, 300, 1)
    a.show()