"""
Field mapping application for calculating the effective dipole moment of an object.
Created by Matthew Barbattini on 12/20/22

Uses the three-axis newmark position robot and the newmark rotation robot.

Dependencies:
PyQt5: UI
pyvisa: VISA for communicating with the magnetometer
NSCG3: a newmark motion controller wrapper to send commands and communicate
TFM1186: a wrapper for the MetroLab TFM1186 vector magnetometer and USB adapter TFM1176
AutoMap: Abstract base class for mapping objects
NonRotationalMap: class for creating mapping objects that do not involve rotations
RotationalMap: class for creating mapping objects that have rotations
LMFIT: a python class for an improvement to the Levenber-Marquardt algorithm
ortools: math functions for route optimization
gclib: driver for communicating with newmark controllers
"""


from auto_map import AutoMap

import numpy as np
import time

class NonRotationalMap(AutoMap):

    def __init__(self):
        super().__init__()

        # self.tempCoords = np.empty((0,3))
        # self.tempObjectB = np.empty((0,3))
        # self.tempBackgroundB = np.empty((0,3))
        self.tempData = np.empty((0,3))

    def xyplane(self, start, end, nPointsX, nPointsY):
        """
        Maps out a plane based on a start point and end point. Can only do horizontal or vertical planes, no tilted planes.
        At least one coordinate has to remain the same to correctly work.
        Inputs:
         start: array<int> [x,y,z] start position
         end: array<int> [x,y,z] end position
         nPointsX: <int> number of points along first dimension
         nPointsY: <int> number of points along second dimension
        """
        
        xStart, yStart, zStart = start
        xEnd, yEnd, zEnd = end

        # xy plane
        if not zStart == zEnd:
            raise RuntimeError("Corners do not have the same z coordinate.")
        else:
            x = np.linspace(xStart, xEnd, nPointsX)
            y = np.linspace(yStart, yEnd, nPointsY)
            z = np.full((nPointsX, nPointsY), zStart)

            X,Y = np.meshgrid(x,y)

            X = X.reshape(1,-1)
            Y = Y.reshape(1,-1)
            Z = z.reshape(1,-1)

            points = np.vstack((X,Y,Z))

            self.points = points

    def sphere(self, radius, nPoints, minHeight):
        """
        Maps out a equally-distributed top sphere above the object. Number of points is not the true value because some are
        cut out in the middle. 
        Inputs:
         radius: <float> in mm
         nPoints: <float> number of points in the sphere
         minHeight: <float> minimum height the sphere will extend to. Corresponds to the slice to create a hat of the sphere.
        """ 

        points = []
        temp = []
        phi = np.pi * (3 - np.sqrt(5.))

        for i in range(nPoints):

            y = 1 - (i / float(nPoints - 1)) * 2

            # scale the algorithm's unit sphere radius by the user's input value
            radius = radius * np.sqrt(1 - y * y)
            theta = phi * i

            x = np.cos(theta) * radius
            z = np.sin(theta) * radius

            # scale y
            y = radius * y

            v = np.vstack((x,y,z))
            temp.append(v[:,:])
        temp = np.hstack(temp)

        zTemp = temp[2,:]

        minHeight = minHeight

        # delete any points between -minHeight and minHeight. Carves out a solid slice in the middle of the sphere
        indices = []
        for i in range(len(zTemp)):
            if zTemp[i] < minHeight and zTemp[i] > -minHeight:
                indices.append(i)
        temp = np.delete(temp, indices, axis=1)

        # reflect points from bottom slice to the top slice. Might be duplicate points
        for i in range(len(temp[2])):
            if temp[2,i] > minHeight:
                temp[2,i] = temp[2,i] * -1

        points = temp

        self.points = points

    def box(self, start, stop, nPointsX, nPointsY, nPointsZ):
        """
        Maps out a solid rectangular box with a specified number of sample points.
        Inputs:
         start: (3,) array, the starting coordiante [x,y,z]
         stop: (3,) array, the ending coordinate [x,y,z]
         nPointsX: <int> number of points along the x axis
         nPointsY: <int> number of points along the y axis
         nPointsZ: <int> number of points along the z axis
        """
        xx = np.linspace(start[0], stop[0], nPointsX)
        yy = np.linspace(start[1], stop[1], nPointsY)
        zz = np.linspace(start[2], stop[2], nPointsZ)
        
        X,Y,Z = np.meshgrid((xx,yy,zz))
        
        X = X.flatten()
        Y = Y.flatten()
        Z = Z.flatten()

        points = np.vstack((X,Y,Z))

        self.points=points


    def processData(self, type):
        """
        Transforms the data into the lab frame and stores it.
        Stores as either a background map or an object map
        """

        # reflect the z coordinate
        self.tempData[:,0] *= -1
        # reflect the by component
        self.tempData[:4] *= -1

        # store to the final arrays
        if type=="background":
            self.backgroundB = self.tempData
        elif type=="object":
            self.objectB = self.tempData
        else:
            print("Type of map (background / object) not defined.")


    def map(self, type, delay=1, speed=30, acceleration=20, deceleration=20, **kwargs):
        """
        Sends sample points to the 3-axis positioner as commands. Changing default values will results in dramatic changes in mapping routines.
        Inputs:
         delay: <int> time delay between each sample points in seconds
         speed: <int> speed of the 3-axis positioner
         acceleration: <int> acceleration of the 3-axis positioner
         deceleration: <int> deceleration of the 3-axis positioner
        """
        self.optimizeTrajectory()
        self.speed = speed
        self.acceleration = acceleration
        self.deceleration = deceleration

        i = 0
        
        while i < self.points.shape[0]:
            try:
                 self.threeAxisRobot.moveAbsolute(self.points[:,i])
                 self.threeAxisRobot.wait()
            except:
                print('Attempting to remeasure point')

            time.sleep(delay)
            self.trigger(i, averages=100)
            i += 1
            print(f"Point {i}")

        if type=="background":
            self.backgroundB = self.tempData
        elif type=="object":
            self.objectB = self.tempData
            

