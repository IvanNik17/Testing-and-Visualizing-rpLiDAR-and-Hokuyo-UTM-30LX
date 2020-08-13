# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 09:08:49 2018

@author: ivan

Helper functions for the LiDAR_animate and Lidar_visualizer. only the pdist from scipy is used, but this can be easily substituted with
numpy implementation. Some of the functions are not fully vectorised, but in their final implementation will be. Some of the functions
are still in development and current versions are still in testing. List functions:
    -calculateMeans - a blanket function for calculating the mean angle and mean distance from the Lidar readings, calls circularMean
    -circle2cart_drone - function for calculating the carthesian coordinates of the lidar
    -circle2cart_points - function for calculating the reprojected blade point coordinates
    -calculateAnglesPCA - function for calculating the orientation of the blade segment using PCA
    -testEllipse - function for creating the ellipse using two diameters, rotation angle and center point coordinates
    -calculateMinMax - function for calculating the major diameter of the ellipse from the highest distance between the detected points
    (currently outputs other distances, which might be useful at later date)
    -intersectionLineCurve - function that calculates the intersection between a curve an a line made from two points. If it's a closed curve
    it calculates the distances from the start point to the intersections and takes the closest one
    -getDataFromIMU - function for getting the data from the serial connection to the IMU/Arduino combo, it takes all the data checks for errors formats it in a list
    
    
 
"""

import math

import numpy as np

import serial
from scipy.spatial.distance import pdist


def calculateMeans(angleDistList):
    # It calculates the means of the distance and angle, it introduces weights, so if needed a weighted average an be easily implemented   
    dists = angleDistList[:,1]
    weight = dists/sum(dists)
    
    meanAngleCalc = circularMean(angleDistList[:,0],weight)    

    meanDistCalc = np.average(angleDistList[:,1],weights= weight)

    return meanAngleCalc, meanDistCalc



def circularMean(angles,weight):
    
    #For calculating the circular mean, the going from 0 to 360 needs to be observed, so the cos and sin angles are calculated
    # as well as the mean sin and cos, which are then used to determine if additional degrees need to be added
    sinAngles = np.sin(np.radians(angles))
    cosAngles = np.cos(np.radians(angles))
    

    meanSin = np.dot(weight,sinAngles)
    meanCos = np.dot(weight,cosAngles)

    
    if (meanSin > 0 and meanCos > 0):
        meanAngle = math.atan(meanSin/meanCos)
        
    elif (meanCos < 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(180)
    elif (meanSin <0 and meanCos> 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(360)
    
    return math.degrees(meanAngle)

# The difference between the two circle2cart functions is the direction of projection of  the data. The points program is currently not
#properly vectoried and it can take an additional angle to rotate it which is not presently used.    
def circle2cart_drone(center,angle, distance):
    xC = center[0] - distance * math.sin(math.radians(angle))
    yC = center[1] - distance * math.cos(math.radians(angle))
    return [xC,yC]

def circle2cart_points(center,angle,distance):
    
    
    xC = center[0] + distance * math.sin(math.radians(angle))
    yC = center[1] + distance * math.cos(math.radians(angle))
    return [xC,yC]

#    xC = []
#    yC = []
#    for i in range(0,len(measurement)):
#        xC.append( circleCenter[0] + measurement[i,1] * math.sin(math.radians(measurement[i,0] + addDegree)) )
#        yC.append( circleCenter[1] + measurement[i,1] * math.cos(math.radians(measurement[i,0] + addDegree)) )
#    
#    return np.column_stack((xC,yC))
    
    
    
def calculateAnglesPCA(bladePointsPos):
    
    # calculate covariance matrix of the detected blade points

    cov_mat = np.cov(bladePointsPos.T)
    # calculate the eigen values and eigen vectors from the covariance matrix
    eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
    
    # find the largest eigenvalue and the eigenvector that coresponds to it
    maxIndEigenval = np.argmax(eig_val_cov)
    evec1 = eig_vec_cov[:,maxIndEigenval]
    
    # calculate the angle from the eigenvector
    angleOffsetMeasured = math.degrees(np.arctan2( evec1[0],evec1[1]   )) 
    
    # currently the algorithm does not detect angles larger than 180 degrees, and it's up to the user to specify if it's suction of pressure side
    if angleOffsetMeasured < 0:
        angleOffsetMeasured =  angleOffsetMeasured + 180
        
    return angleOffsetMeasured
    
    
    
def testEllipse(distance1,distance2, orientation, center):
    # create ellipse using diameters, orientation and centers.
    # The algorithm also calculates the radius of each point of the ellipse and outputs together with the angle - currently both not used
    numberOfPoints = 360
    centerX = center[0]
    centerY = center[1]
    orientation = -orientation
    theta = np.linspace(0, 2*math.pi, numberOfPoints)
    orientation=orientation*math.pi/180
    
    radiusDist = []
    xx2 = []
    yy2 = []

    for i in range(0,numberOfPoints):
        xx = -(distance1/2) * math.sin(theta[i]) + centerX
        yy = -(distance2/2) * math.cos(theta[i]) + centerY
    
        xx2_temp = (xx-centerX)*math.cos(orientation) - (yy-centerY)*math.sin(orientation) + centerX
        yy2_temp = (xx-centerX)*math.sin(orientation) + (yy-centerY)*math.cos(orientation) + centerY

        xx2.append(xx2_temp)
        yy2.append(yy2_temp)
    
        radiusDist.append(math.sqrt(xx2_temp**2 + yy2_temp**2))


    degrees = range(0,numberOfPoints)
  
    radiusAngles = np.column_stack((radiusDist,degrees))
    ellipsePos = np.column_stack((xx2,yy2))

    return radiusAngles, ellipsePos
    
def calculateMinMax(pointCloud2D):
    # get all distances between the blade points
    distAll = np.linalg.norm(pointCloud2D - pointCloud2D[:,None], axis=-1)
    # remove duplicates
    distAll_noDup = np.triu(distAll)
    # get the maximum distances, minimum distances, as well as the points which have the minimum and maximum distances
    minDist = np.min(distAll_noDup[np.nonzero(distAll_noDup)])
    maxDist = np.max(distAll_noDup[np.nonzero(distAll_noDup)])
    minDist_index = np.where(distAll_noDup==minDist)
    minDist_points = np.array([pointCloud2D[minDist_index[0][0],:], pointCloud2D[minDist_index[1][0],:]])
    maxDist_index = np.where(distAll_noDup==maxDist)
    maxDist_points = np.array([pointCloud2D[maxDist_index[0][0],:], pointCloud2D[maxDist_index[1][0],:]])
    
    return minDist, maxDist, minDist_points, maxDist_points
    
    
def intersectionLineCurve(lineFirst, lineSecond, curvePoints):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = np.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    
    
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = np.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings
    
    distToLidar_1 = np.array([ lineSecond,  cross_points[0,:] ]) # in case of a closed curve calculate distance to lidar
    distToLidar_1 = pdist(distToLidar_1,'euclidean')
    
    distToLidar_2 = np.array([ lineSecond,  cross_points[1,:] ])
    distToLidar_2 = pdist(distToLidar_2,'euclidean')
    
    if (distToLidar_1 > distToLidar_2): # get the closest value
        outputInters = cross_points[1,:]
    else:
        outputInters = cross_points[0,:]
    

    return outputInters
       
    
    
    
    
# Get IMU data   
def getDataFromIMU(ser, oldRead):
    
    # get previous returned values in case that new values are corrupt
    returnedList = oldRead
    try:
        if (ser.inWaiting()>0): # wait until something comes at the serial
            
            try:        
                arduinoLine = ser.readline().decode().strip() # read the whole line , decode it and strip it

                arduinoLine_split = arduinoLine.split('|') # split it into a list of values
                if len(arduinoLine_split) == 7: # check that there are exactly 7 values
                    try:
                        arduinoLine_floats = [round(float(x),1) for x in arduinoLine.split('|')] # make each of the list entries into floats
                        returnedList = arduinoLine_floats 
                        ser.flushInput() # flush the input buffer
                    except ValueError: # in case the values inside cannot be converted into floats pass
                        pass
            except UnicodeDecodeError: # in case of decoding error pass
                pass
    except IOError:
        print("IO error")

    return returnedList