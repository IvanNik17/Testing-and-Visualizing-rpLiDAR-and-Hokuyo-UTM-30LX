# Testing and Visualizing rpLiDAR and Hokuyo UTM-30LX
 Python code for testing and visualizing the output of two types of LiDARs

The code repository contains the core libraries for both the rpLiDAR and the UTM-30LX LiDARs in Python. The code can be run from two scripts:
1. LiDAR_visualize - this script captures a ful rotation of the LiDAR and visualizes it
2. LiDAR_animate - this visualizes in real time the output of the LiDAR

Both of these testing scripts can be as a starting point for implementing capturing and visualization of LiDAR data for positioning, scanning, SLAM, etc.
The helperFunctions_lite.py contains a number of fuctions for self positioning, orientation and visualization of the LiDAR 
