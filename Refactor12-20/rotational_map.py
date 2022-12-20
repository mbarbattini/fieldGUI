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
import math

class RotationalMap(AutoMap):

    def __init__(self, nRotations):
        super().__init__()

        # self.tempCoords = np.empty((0,3))
        # self.tempObjectB = np.empty((0,3))
        # self.tempBackgroundB = np.empty((0,3))
        # temporary data for each run of the rotational routine
        self.tempData = np.empty((0,3))
        # temp points of all rotations. Not self.points
        # self.points does not change because the shape mapped is always in the same locations
        self.tempPoints = np.empty((0,3))
        self.nRotations = nRotations


    def xyplane(self, start, end, nPointsX, nPointsY):
        """
        
        """
        xStart, yStart, zStart = start
        xEnd, yEnd, zEnd = end

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
        pass

    def box(self, start, stop, nPointsX, nPointsY):
        pass

    def processData(self, currentRotation, type):
        """
        Processes data by tansforming the measured data according to the current rotation. 
        
        Parameters:
        currentRotation: int
            the number of the current rotation i.e. 0-5 for 6 total rotational maps of the geometry
        type: str
            1) "background"
            2) "object"
        """ 

        def rotationX(angle):
            angle = math.radians(angle)
            return np.array(([1 ,       0       ,       0        ],
                            [0 , np.cos(angle)  , -np.sin(angle) ],
                            [0 , np.sin(angle) ,  np.cos(angle) ]), dtype=float)
        
        angle = 360 // self.nRotations

        # transform to lab frame
        # reflect z coord
        self.points[:,2] *= -1
        # reflect By
        self.tempData[:,1] *= -1

        # rotate each vector by the rotation matrix for the calculated angle
        # first run does not need to be transformed
        if currentRotation != 0:
            for i in range(self.points.shape[0]):
                # position vector
                self.tempPoints[i] = rotationX(currentRotation * angle) @ self.points[i,:]
                # field vector
                self.tempData[i] = rotationX(currentRotation * angle) @ self.tempData[i,:]

        # update the final data arrays with the data for the current rotation
        # i.e. append the temp data to the final data
        if type == "background":
            self.backgroundB = np.vstack((self.backgroundB, self.tempData))
        elif type == "object":
            self.objectB = np.vstack((self.objectB, self.finalField))
        self.finalPoints = np.vstack((self.finalPoints, self.tempPoints))

    def map(self, type, delay=1, speed=30, acceleration=20, deceleration=20, **kwargs):
        """
        The mapping routine. Maps the field n times, where n is the number of rotations.
        Saves the data of all maps into the final arrays 
        
        Parameters:
        type: str
            1) "background"
            2) "object" 
        """
        self.optimizeTrajectory()
        self.speed = speed
        self.acceleration = acceleration
        self.deceleration = deceleration

        # index for visiting the points
        i = 0

        # add the first set of points to finalPoints
        self.finalPoints = np.vstack((self.finalPoints, self.points))

        for nRot in range(self.nRotations):
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

            # process the data for the current rotation
            self.processData(nRot, type)
            # increment the number of rotations
            nRot += 1
            print(f"\n\nRotation {nRot}\n\n")
            # rotate the robot
            self.rotateRobot()
            # reset the index for the points
            i = 0

    def rotateRobot(self, speed=30, acceleration=20, deceleration=30):
        """
        Rotates the robot.
        Assumes that it starts at zero position that is parallel to the ground, which is done manually.
        """
        rotationalRange1Revolution = 575
        distance = [-rotationalRange1Revolution / self.nRotations]

        self.rotationalRobot.speed = speed
        self.rotationalRobot.acceleration = acceleration
        self.rotationalRobot.deceleration = deceleration

        self.rotationalRobot.moveRelative(distance)
        self.rotationalRobot.wait()
