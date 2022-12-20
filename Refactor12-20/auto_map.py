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

from Assets.nscg3 import NSCG3
from Assets.tfm1186 import TFM1186
import numpy as np
import time
from abc import ABC, abstractmethod
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from scipy.spatial import distance_matrix
from lmfit import Parameters
from lmfit import Model
from scipy.optimize import root
import os

class AutoMap(ABC):

    def __init__(self):
        self.threeAxisRobot = NSCG3("169.254.154.15", "XYZ", simulate=True)
        self.rotationalRobot = NSCG3("192.169.20.99", "XYZ", simulate=True)

        resources = TFM1186.GetResources()
        self.tfm = TFM1186(resources[0], simulate=True)

        # the final data for calculations and exporting
        self.objectB = np.empty((0,3))
        self.backgroundB = np.empty((0,3))
        self.finalPoints = np.empty((0,3))        

        # points sent to the mapping robot
        self.points = np.empty((0,3))

        # magnetic field data for calculating dipole moment
        self.B = np.empty((0,3))
    

    def optimizeTrajectory(self):
        """
        Optimizes trajectory to minimize the total travel distance. Determining
        the shortest path to sample all points is a traveling salesman problem.
        A global optimum is not gaurenteed, and for structured sampling schemes,
        this function may actually make the trajectory longer. However, for
        unstructured sampling schemes, an improvement of 2-3x can usually be
        achieved.
        Inputs:
         None
        Outputs:
         None
        """
        print('Optimize Trajectory called.')
        # Distance callback
        def create_distance_callback(dist_matrix):
            # Create a callback to calculate distances between cities.
            def distance_callback(from_node, to_node):
                return dist_matrix[from_node][to_node]
            return distance_callback

        # Compute the distance between all points
        dist_matrix = distance_matrix(self.points.transpose(),self.points.transpose()).tolist()

        # parameters needed for ortools tsp optimization
        num_routes = 1
        depot = 0
        tsp_size = self.points.shape[1]

        routing = pywrapcp.RoutingModel(tsp_size,num_routes,depot)
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        # guided local search will let the optimizer wander out of a local
        # minimum. A search time limit of 30s will be given
        search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit_ms = 30000
        dist_callback = create_distance_callback(dist_matrix)
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
        assignment = routing.SolveWithParameters(search_parameters)

        # Extract the optimal trajectory
        index = routing.Start(0)
        route = list()
        while not routing.IsEnd(index):
            route.append(index)
            index = assignment.Value(routing.NextVar(index))

        # Reorder the points according to the optimal trajectory
        self.points = self.points[:,route]

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

    def trigger(self, index, **kwargs):
        """
        Measure the magnetic field at the robot's current position.
        Updates the data point-by-point.
        """
        averages = 100
        for key,value in kwargs.items():
            if key.lower() == 'averages':
                averages = value

        Bx,sat = self.tfm.measureBlock(averages,'X')
        By,sat = self.tfm.measureBlock(averages,'Y')
        Bz,sat = self.tfm.measureBlock(averages,'Z')
        Bx = np.average(Bx)
        By = np.average(By)
        Bz = np.average(Bz)
        B = np.array([Bx,By,Bz])
        self.tempData[index,:] = np.hstack((self.points[:, index].transpose(), B))
    
    def saveData(self, foldername, backgroundSubtract):
        """
        Saves data to cwd/Data
        
        Parameters:
        foldername: str
            The folder in cwd/Data where data is stored
        backgroundSubtract: bool
            whether the current run is a background subtraction
        """
        if backgroundSubtract:
            currentDirectory = os.getcwd()
            filenameObject = f"{currentDirectory}\\Data\\{foldername}\\_object.csv"
            filenameBackground = f"{currentDirectory}\\Data\\{foldername}\\_background.csv"
            filenameSubtracted = f"{currentDirectory}\\Data\\{foldername}\\_subtracted.csv"
            filenameCoord = f"{currentDirectory}\\Data\\{foldername}\\_coords.csv"

            # create the empty files
            os.makedirs(os.path.dirname(filenameObject), exist_ok=True) 
            os.makedirs(os.path.dirname(filenameBackground), exist_ok=True)
            os.makedirs(os.path.dirname(filenameCoord), exist_ok=True)
            os.makedirs(os.path.dirname(filenameSubtracted), exist_ok=True)

            # save the data to the files
            np.savetxt(filenameObject, self.objectB, delimiter=',')
            np.savetxt(filenameBackground, self.backgroundB, delimiter=',')
            np.savetxt(filenameSubtracted, self.objectB - self.backgroundB, delimiter=',')
            np.savetxt(filenameCoord, self.finalPoints, delimiter=',') 
        else:
            currentDirectory = os.getcwd()
            filenameObject = f"{currentDirectory}\\Data\\{foldername}\\_object.csv"
            filenameCoord = f"{currentDirectory}\\Data\\{foldername}\\_coords.csv"
            
            # create the empty files
            os.makedirs(os.path.dirname(filenameObject), exist_ok=True) 
            os.makedirs(os.path.dirname(filenameCoord), exist_ok=True)

            # save the data to the files
            np.savetxt(filenameObject, self.objectB, delimiter=',')
            np.savetxt(filenameCoord, self.finalPoints, delimiter=',')

    def calculateDipoleMoment(self, method, type):
        """
        Calculates the effect dipole moment of the measured data 
        
        Parameters:
            method: bool
                0: no background subtraction
                1: background subtraction
            type: bool
                0: lm algorithm
                1: lfmit algorithm
        """
        if type == 1:
            self.B = self.objectB - self.backgroundB
        elif type == 0:
            self.B = self.objectB

        if method == 0:
            def vector_func(x):
            
                a = x[0]
                b = x[1]
                c = x[2]
                mx = x[3]
                my = x[4]
                mz = x[5]
                
                bx = self.B[:,0]
                by = self.B[:,1]
                bz = self.B[:,2]
                xPos = self.coords[:,0]
                yPos = self.coords[:,1]
                zPos = self.coords[:,2]

                equations = []
                
                for i in range(len(bx)):
                    equations.append(
                        bx[i] - (10**(-7))*((3*(mx*(xPos[i]-a)+my*(yPos[i]-b)+mz*(zPos[i]-c))*(xPos[i]-a)/((xPos[i]-a)**2+(yPos[i]-b)**2+(zPos[i]-c)**2)**(5/2))-(mx/((xPos[i]-a)**2+(yPos[i]-b)**2+(zPos[i]-c)**2)**(3/2)))
                    )

                for i in range(len(by)):
                    equations.append(
                        by[i] - (10**(-7))*((3*(mx*(xPos[i]-a)+my*(yPos[i]-b)+mz*(zPos[i]-c))*(yPos[i]-b)/((xPos[i]-a)**2+(yPos[i]-b)**2+(zPos[i]-c)**2)**(5/2))-(my/((xPos[i]-a)**2+(yPos[i]-b)**2+(zPos[i]-c)**2)**(3/2)))
                    )

                for i in range(len(bz)):
                    equations.append(
                        bz[i] - (10**(-7))*((3*(mx*(xPos[i]-a)+my*(yPos[i]-b)+mz*(zPos[i]-c))*(zPos[i]-c)/((xPos[i]-a)**2+(yPos[i]-b)**2+(zPos[i]-c)**2)**(5/2))-(mz/((xPos[i]-a)**2+(yPos[i]-b)**2+(zPos[i]-c)**2)**(3/2)))
                    )
                
                return equations


            # an initial guess vector
            # [location x, location y, location z, magnitude x, magnitude y, magnitude z]
            x0 = np.array[0.05, 0.05, 0.05, 1.2, 1.2, 1.2]

            sol = root(vector_func, x0, method='lm', options={'xtol': 1.49012e-20, 'ftol': 1.49012e-20})
            
            # if the value is approximatley 0
            tol = 1e-6
            if sol.x[0] < tol:
                sol.x[0] = 0
            if sol.x[1] < tol:
                sol.x[1] = 0
            if sol.x[2] < tol:
                sol.x[2] = 0
            if sol.x[4] < tol:
                sol.x[4] = 0
            if sol.x[5] < tol:
                sol.x[5] = 0

            mag = np.sqrt(sol.x[3]**2+sol.x[4]**2+sol.x[5]**2)

            finalString = ""
            finalString.append(f"x[0] = a  = {sol.x[0]} m\n")
            finalString.append(f"x[1] = b  = {sol.x[1]} m\n")
            finalString.append(f"x[2] = c  = {sol.x[2]} m\n")
            finalString.append(f"x[3] = mx = {sol.x[3]}\n")
            finalString.append(f"x[4] = my = {sol.x[4]}\n")
            finalString.append(f"x[5] = mz = {sol.x[5]}\n")
            finalString.append(f"\nmagnitude: {mag} Am^2\n")

            return finalString

        elif method=="lmfit":
            """
            Uses the lmfit algorithm.
            """
            def magField(d,x,y,z,a,b,c,mx,my,mz):
                # magnetic field equations
                bx = (3*(mx*(x-a)+my*(y-b)+mz*(z-c))*(x-a)/(((x-a)**2+(y-b)**2+(z-c)**2)**(5/2))-mx/(((x-a)**2+(y-b)**2+(z-c)**2)**(3/2)))*10**(-7)
                
                by = (3*(mx*(x-a)+my*(y-b)+mz*(z-c))*(y-b)/(((x-a)**2+(y-b)**2+(z-c)**2)**(5/2))-my/(((x-a)**2+(y-b)**2+(z-c)**2)**(3/2)))*10**(-7)

                bz = (3*(mx*(x-a)+my*(y-b)+mz*(z-c))*(z-c)/(((x-a)**2+(y-b)**2+(z-c)**2)**(5/2))-mz/(((x-a)**2+(y-b)**2+(z-c)**2)**(3/2)))*10**(-7)

                # array of either 0 or 1 based on the x,y,z component
                bxBooleanArray = np.where(d == 1.0, 1.0, 0.0)
                byBooleanArray = np.where(d == 2.0, 1.0, 0.0)
                bzBooleanArray = np.where(d == 3.0, 1.0, 0.0)

                # multiply the corresponding equation by the boolean value
                finalBx = bx * bxBooleanArray
                finalBy = by * byBooleanArray
                finalBz = bz * bzBooleanArray

                # reshape the (3N,3) array into a (3N,) array. Adding a non-zero value to two zero values for each row.
                finalArray = np.add(finalBx,finalBy, dtype=float)
                finalArray = np.add(finalArray, finalBz, dtype=float)

                return finalArray

            xData = np.vstack((self.coords, self.B[:,0]))
            yData = np.vstack((self.coords, self.B[:,1]))
            zData = np.vstack((self.coords, self.B[:,2]))

            # add the value for d for each component in a new column
            # x: d=1, y: d=2, z: d=3
            xAddedD = np.hstack((xData, np.full((xData.shape[0], 1), 1.0)))
            yAddedD = np.hstack((yData, np.full((yData.shape[0], 1), 2.0)))
            zAddedD = np.hstack((zData, np.full((zData.shape[0], 1), 3.0)))

            dataSeparated = np.vstack((xAddedD, yAddedD, zAddedD))

            xFinal = dataSeparated[:,0]
            yFinal = dataSeparated[:,1]
            zFinal = dataSeparated[:,2]
            magFieldDataFinal = dataSeparated[:,3]
            dFinal = dataSeparated[:,4]

            dipoleModel = Model(magField, independent_vars=['x', 'y', 'z', 'd'])

            params = Parameters()
            params.add('a',value=0.01, min=-1.0, max=1.0)
            params.add('b',value=0.01, min=-1.0, max=1.0)
            params.add('c',value=0.01, min=-1.0, max=1.0)
            params.add('mx',value=0.5, min=-10.0, max=10.0)
            params.add('my',value=0.5, min=-10.0, max=10.0)
            params.add('mz',value=0.5, min=-10.0, max=10.0)

            result = dipoleModel.fit(magFieldDataFinal, params, x=xFinal, y=yFinal, z=zFinal, d=dFinal)
            return result.fit_report()

    @abstractmethod
    def map(self, type, delay=1, speed=30, acceleration=20, deceleration=20, **kwargs):
        pass


