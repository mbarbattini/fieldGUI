import sys
import os
from functools import reduce
[sys.path.append(i) for i in['.','..']]
l=[]
script_path=os.path.split(sys.argv[0])
for i in range(len(script_path)):
    sys.path.append( reduce(os.path.join,script_path[:i+1]))
from abc import ABC, abstractmethod
import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from scipy.spatial import distance_matrix
import time
from Assets.tfm1186 import TFM1186
from Assets.nscg3 import NSCG3
import pandas as pd
import math
from scipy.optimize import root
# from mayavi import mlab
 
class AutoMap(ABC):
    """
    Automatic mapping routine for the newmark 3-axis stage. AutoMap is an
    abstract class with abstract method "trigger." AutoMap must be subclassed
    and the trigger method defined in order to be used.

    All distance measurements are in mm and all magnetic field data is in nT.
    """

    def __init__(self,addr='169.254.154.15',simulate=False):
        """
        AutoMap constructor.  Initializes connection to the stage
        Inputs:
         addr: <string> IP address of the newmark motion controller
         simulate: <bool> simulate hardware if not connected
        Outputs:
         None
        """
        super().__init__()
        self.points = None
        self.threeAxisNewmark = NSCG3(addr,'XYZ',simulate=simulate)
        self.temp_filename = None
        self.main_filename = None





    def line(self,pStart,pStop,numPoints):
        """
        Creates a line trajectory from pStart to pStop with uniform spacing
        Inputs:
         pStart: <list> x,y,z coordinates of start point
         pStop: <list> x,y,z coordinates of stop point
         numPoints: <list> number of points to sample along line
        Outputs:
         None
        """
        xStart,yStart,zStart = pStart
        xStop,yStop,zStop = pStop
        points = np.ndarray((3,numPoints))
        points[0,:] = np.linspace(xStart,xStop,numPoints)
        points[1,:] = np.linspace(yStart,yStop,numPoints)
        points[2,:] = np.linspace(zStart,zStop,numPoints)
        self.points = points





    def box(self,pStart,pStop,nPoints1D,nPoints2D,nPoints3D):
        """
        Maps out a solid rectangular box with a specified number of sample points.
        Inputs:
         pStart: array<int> the starting coordiante [x,y,z]
         pStop: array<int> the ending coordinate [x,y,z]
         nPoints1D: <int> number of points along the x axis
         nPoints2D: <int> number of points along the y axis
         nPoints3D: <int> number of points along the z axis
        Outputs:
         None
        """
        # divide the length of the x,y,z axes into the correct number of points
        pStart=np.array(pStart)
        pStop=np.array(pStop)
        axisLength=pStop[0]-pStart[0]
        #axisLength=abs(axisLength)
        nPoints1D=np.linspace(0,axisLength,nPoints1D)
        axisLength=pStop[1]-pStart[1]
        #axisLength=abs(axisLength)
        nPoints2D=np.linspace(0,axisLength,nPoints2D)
        axisLength=pStop[2]-pStart[2]
        #axisLength=abs(axisLength)
        nPoints3D=np.linspace(0,axisLength,nPoints3D)
        
        X,Y= np.meshgrid(nPoints1D,nPoints2D)
        
        # X and Y are reshaped to a 1D array
        X=X.reshape(1,-1)
        Y=Y.reshape(1,-1)
        # Z is a 10 x 10 tuple, basically number of data points 
        Z=(len(nPoints1D),len(nPoints2D))
        # Z is full of 0s in the size of the previous, flattened to a 1D array
        Z=np.zeros(Z)
        Z=Z.reshape(1,-1)
        test=[]
        # this is where all the math for increasing the height of the box happens.
        # looks to be a solid box
        for i in range(len(nPoints1D)):
            t=np.add(Z,nPoints3D[i])
            v=np.vstack((X,Y,t))
            test.append(v[:,:])
        test=np.hstack(test)
        points=test
        # add on the start point in the correct dimension
        points=points+pStart.reshape(3,1)
        # final matrix is shape (3, # of points) Ex. (3,1000). X Y Z are rows, cols are data points
    
        # assign it to the AutoMap object, then inherited by TestMap
        self.points=points





    def plane(self, start, end, nPoints1D, nPoints2D):
        """
        Maps out a plane based on a start point and end point. Can only do horizontal or vertical planes, no tilted planes.
        At least one coordinate has to remain the same to correctly work.
        Inputs:
         start: array<int> [x,y,z] start position
         end: array<int> [x,y,z] end position
         nPoints1D: <int> number of points along first dimension
         nPoints2D: <int> number of points along second dimension
        Outputs:
         None
        """
        
        points = []
        
        xStart, yStart, zStart = start
        xEnd, yEnd, zEnd = end

        x = []
        y = []
        z = []

        # z is the dimension that doesn't change, based on input of points
        # yz plane
        if xStart == xEnd:
            x = np.full((nPoints1D,nPoints2D), xStart)
            y = np.linspace(yStart, yEnd, nPoints1D)
            z = np.linspace(zStart, zEnd, nPoints2D)

            Y,Z = np.meshgrid(y,z)

            X = x.reshape(1,-1)
            Y = Y.reshape(1,-1)
            Z = Z.reshape(1,-1)

            points = np.vstack((X,Y,Z))

            self.points = points
            return

        # xz plane
        if yStart == yEnd:
            x = np.linspace(xStart, xEnd, nPoints1D)
            y = np.full((nPoints1D, nPoints2D), yStart)
            z = np.linspace(zStart, zEnd, nPoints2D)

            X,Z = np.meshgrid(x,z)

            X = X.reshape(1,-1)
            Y = y.reshape(1,-1)
            Z = Z.reshape(1,-1)

            points = np.vstack((X,Y,Z))

            self.points = points
            return

        # xy plane
        if zStart == zEnd:
            x = np.linspace(xStart, xEnd, nPoints1D)
            y = np.linspace(yStart, yEnd, nPoints2D)
            z = np.full((nPoints1D, nPoints2D), zStart)

            X,Y = np.meshgrid(x,y)

            X = X.reshape(1,-1)
            Y = Y.reshape(1,-1)
            Z = z.reshape(1,-1)

            points = np.vstack((X,Y,Z))

            self.points = points
            return
        
        print("Start and end points do not create a horizontal or vertical plane.")
        return





    def sphere(self, _radius, _nPoints, _minHeight):
        """
        Maps out a equally-distributed top sphere above the object. Number of points is not the true value because some are
        cut out in the middle. 
        Inputs:
         _radius: <int> in mm
         _nPoints: <int> number of points in the sphere
         _minHeight: <float> minimum height the sphere will extend to. Corresponds to the slice to create a hat of the sphere.
        Outputs:
         None
        """ 

        points = []
        temp = []
        phi = np.pi * (3 - np.sqrt(5.))

        for i in range(_nPoints):

            y = 1 - (i / float(_nPoints - 1)) * 2

            # scale the algorithm's unit sphere radius by the user's input value
            radius = _radius * np.sqrt(1 - y * y)
            theta = phi * i

            x = np.cos(theta) * radius
            z = np.sin(theta) * radius

            # scale y
            y = _radius * y

            v = np.vstack((x,y,z))
            temp.append(v[:,:])
        temp = np.hstack(temp)

        zTemp = temp[2,:]

        minHeight = _minHeight

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
    
    



    def visualize(self):
        """
        Plots a vector field of the measusred data in MayaVI. 
        Inputs:
         None
        Outputs:
         Vector plot in MayaVI
        """
        df = pd.read_csv(r"%s" %self.main_filename)

        x = df['X']
        y = df['Y']
        z = df['Z']
        bx = df['Bx']
        by = df['By']
        bz = df['Bz']

        x = x.to_numpy()
        y = y.to_numpy()
        z = z.to_numpy()
        bx = bx.to_numpy()
        by = by.to_numpy()
        bz = bz.to_numpy()

        bx /= 1e9
        by /= 1e9
        bz /= 1e9

        mlab.figure('Vector Field')
        mlab.quiver3d(x,y,z,bx,by,bz, colormap='gist_rainbow')
        mlab.axes(extent=[-300,300,-300,300,-300,300])
        mlab.outline()
        mlab.show()





    def LMalgorithm(self, _initialGuesses):
        """
        Estimates the values of the position (a,b,c) and the components (mx, my, mz) of the 
        measured magnetic dipole using the Levenberg-Marquardt algorithm for non-linear least squares problems.
        Inputs:
         _initialGuesses: array<float> initial guesses supplied to the algorithm, needs 6 entries [a,b,c,mx,my,mz]
        Outputs:
         Prints the parameters and the estimated values. 
        """

        df = pd.read_csv(r"%s" %self.main_filename)

        # store data in dataframes
        bx = df['Bx']
        by = df['By']
        bz = df['Bz']
        x_pos = df['X']
        y_pos = df['Y']
        z_pos = df['Z']

        # convert to numpy arrays
        bx = bx.to_numpy()
        by = by.to_numpy()
        bz = bz.to_numpy()
        x_pos = x_pos.to_numpy()
        y_pos = y_pos.to_numpy()
        z_pos = z_pos.to_numpy()

        # convert field to Tesla, position to meters.
        bx /= 1e9
        by /= 1e9
        bz /= 1e9
        x_pos /= 1e3
        y_pos /= 1e3
        z_pos /= 1e3

        def vector_func(x):

            a = x[0]
            b = x[1]
            c = x[2]
            mx = x[3]
            my = x[4]
            mz = x[5]

            equations = []

            for i in range(len(bx)):
                equations.append(
                    bx[i] - (10**(-7))*((3*(mx*(x_pos[i]-a)+my*(y_pos[i]-b)+mz*(z_pos[i]-c))*(x_pos[i]-a)/((x_pos[i]-a)**2+(y_pos[i]-b)**2+(z_pos[i]-c)**2)**(5/2))-(mx/((x_pos[i]-a)**2+(y_pos[i]-b)**2+(z_pos[i]-c)**2)**(3/2)))
                )

            for i in range(len(by)):
                equations.append(
                    by[i] - (10**(-7))*((3*(mx*(x_pos[i]-a)+my*(y_pos[i]-b)+mz*(z_pos[i]-c))*(y_pos[i]-b)/((x_pos[i]-a)**2+(y_pos[i]-b)**2+(z_pos[i]-c)**2)**(5/2))-(my/((x_pos[i]-a)**2+(y_pos[i]-b)**2+(z_pos[i]-c)**2)**(3/2)))
                )

            for i in range(len(bz)):
                equations.append(
                    bz[i] - (10**(-7))*((3*(mx*(x_pos[i]-a)+my*(y_pos[i]-b)+mz*(z_pos[i]-c))*(z_pos[i]-c)/((x_pos[i]-a)**2+(y_pos[i]-b)**2+(z_pos[i]-c)**2)**(5/2))-(mz/((x_pos[i]-a)**2+(y_pos[i]-b)**2+(z_pos[i]-c)**2)**(3/2)))
                )
            
            return equations

        def print_solution(sol):

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

            print(f"x[0] = a  = {sol.x[0]} m")
            print(f"x[1] = b  = {sol.x[1]} m")
            print(f"x[2] = c  = {sol.x[2]} m")
            print(f"x[3] = mx = {sol.x[3]}")
            print(f"x[4] = my = {sol.x[4]}")
            print(f"x[5] = mz = {sol.x[5]}")

            mag = np.sqrt(sol.x[3]**2+sol.x[4]**2+sol.x[5]**2)

            print(f"\nmagnitude: {mag} Am^2")

        x0 = np.array(_initialGuesses)
        sol = root(vector_func, x0, method='lm', options={'xtol': 1.49012e-20, 'ftol': 1.49012e-20})
        print_solution(sol)

        



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


    @property
    def numPoints(self):
        return self.points.shape[1]


    @property
    def trajectoryDistance(self):
        dist = 0
        for i in range(1,self.numPoints):
            dist+= np.linalg.norm(self.points[:,i]-self.points[:,i-1])
        return dist




    def plotTrajectory(self):
        """
        Plots the trajectory through the sample points of the 3-axis positioner in matplotlib.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        X = self.points[0,:]
        Y = self.points[1,:]
        Z = self.points[2,:]

        ax.plot3D(X,Y,Z,'-o')
        # ax.set_aspect('equal')

        max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0

        mid_x = (X.max()+X.min()) * 0.5
        mid_y = (Y.max()+Y.min()) * 0.5
        midheight = (Z.max()+Z.min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        # ax.setheightlim(midheight - max_range, midheight + max_range)

        ax.set_xlabel('x [mm]')
        ax.set_ylabel('y [mm]')
        # ax.setheightlabel('z [mm]')
        ax.set_title('Distance = ' + str(self.trajectoryDistance) + 'mm')
        plt.show()





    def plotPoints(self):
        """
        Plots the sample points in matplotlib.
        """
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        for i in range(self.points.shape[1]):
            x = self.points[0,i]
            y = self.points[1,i]
            z = self.points[2,i]
            ax.scatter(x,y,z, marker='.', color='b')

        ax.set_xlabel('x (mm)')
        ax.set_ylabel('y (mm)')
        ax.set_zlabel('z (mm)')

        plt.title('Sample Points')
        plt.show()





    def map(self, delay=1,speed=30,acceleration=20,deceleration=20, **kwargs):
        """
        Sends sample points to the 3-axis positioner as commands. Changing default values will results in dramatic changes in mapping routines.
        Inputs:
         delay: <int> time delay between each sample points in seconds
         speed: <int> speed of the 3-axis positioner
         acceleration: <int> acceleration of the 3-axis positioner
         deceleration: <int> deceleration of the 3-axis positioner
        """
        # self.mapStarted(**kwargs)
        self.speed = speed
        self.acceleration = acceleration
        self.deceleration = deceleration

        i = 0
        
        while i <  self.numPoints:
            try:
                 self.threeAxisNewmark.moveAbsolute(self.points[:,i])
                 self.threeAxisNewmark.wait()
            except:
                print('Attempting to remeasure point')

            time.sleep(delay)
            self.trigger(i, averages=100)
            i += 1
            print("Point " + str(i))
        # self.mapEnded(**kwargs)



    @abstractmethod
    def trigger(self,index,**kwargs):
        pass

    # @abstractmethod
    # def mapStarted(self,**kwargs):
        # pass

    # @abstractmethod
    # def mapEnded(self,**kwargs):
        # pass









