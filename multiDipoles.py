"""
This script is used to test many dipoles against a dataset that likely contains higher order terms.
Each test dipoles is placed in a solid volume at a known location and its magnitude is determined.
Areas with similar magnitudes are an approximation of an actual dipole in the object.
(nMeasurements, 3*nDipoles) matrix size
"""

from __future__ import division
from re import S
import numpy as np
from numpy.core.defchararray import lower
import pandas as pd
import time
from mayavi import mlab
from itertools import product
import multiprocessing as mp
import matplotlib.pyplot as plt


class Dipole:

    def __init__(self, x, y, z):
        
        self.x = x
        self.y = y
        self.z = z
        self.mx = None
        self.my = None
        self.mz = None
        self.mag = None


class MultipleDipoles:

    def __init__(self, filename, scale=False, excel=False):

        self.filename = filename
        self.excel = excel
        self.scale = scale

        if self.excel:
            df = pd.read_excel(self.filename)
        else:
            df = pd.read_csv(self.filename)

        bx = df['Bx']
        by = df['By'] 
        bz = df['Bz']
        xLocations = df['X']
        yLocations = df['Y']
        zLocations = df['Z']

        bx = bx.to_numpy()
        by = by.to_numpy()
        bz = bz.to_numpy()
        xLocations = xLocations.to_numpy()
        yLocations = yLocations.to_numpy()
        zLocations = zLocations.to_numpy()

        if self.scale:
            bx /= 1e9
            by /= 1e9
            bz /= 1e9

            xLocations /= 1e3
            yLocations /= 1e3
            zLocations /= 1e3

        self.totalDipoles = None
        self.dipole = None
        self.xLocations = xLocations
        self.yLocations = yLocations
        self.zLocations = zLocations
        self.magVector = np.hstack((bx,by,bz))
        self.nMeasurement = len(bx)
        self.X = None
        self.Y = None

    def calculate_bx(self, procnum, return_dict):
        """
        Calculates the matrix for all bx measurements.
        Inputs:
        data: <dictionary> all relevant data variables
        procnum: <int> process number, used by multiprocessing
        return_dict: <dictionary> shared variable between processes, contains the final matrices for Bx, By, Bz
        Outputs:
        None
        """

        # the final matrix for Bx, shape (nMeasurements, 3 * self.nDipoles)
        A = np.zeros((self.nMeasurement, self.totalDipoles * 3))

        # find the coefficients
        for m in range(self.nMeasurement):

            # block is an array for each measurement tested against all dipoles, shape (1, 3 * self.nDipoles)
            block = np.empty((1,self.totalDipoles * 3))

            # measurement location (x,y,z)
            x = self.xLocations[m]
            y = self.yLocations[m]
            z = self.zLocations[m]

            for i in range(self.totalDipoles):

                # dipole location (a,b,c)
                a = self.dipole[i].x
                b = self.dipole[i].y
                c = self.dipole[i].z

                mx = (2*(a-x)**2-1*(b-y)**2-1*(c-z)**2) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))
                my = (3*(a-x)*(b-y)) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))
                mz = (3*(a-x)*(c-z)) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))

                comp = np.hstack((mx,my,mz))

                # add the measurement to the block horizontally
                block[0,i*3:i*3+3] = comp

            # insert the block into the correct location in the final matrix for each measurement
            A[m,:] = block

        # multiply by the magnetic permeability constant
        A *= 1e-7

        return_dict[procnum] = A


    def calculate_by(self, procnum, return_dict):
        """
        Calculates the matrix for all by measurements.
        Inputs:
         procnum: <int> process number, used by multiprocessing
         return_dict: <dictionary> shared variable between processes, contains the final matrices for Bx, By, Bz
        Outputs:
         None
        """

        A = np.zeros((self.nMeasurement, self.totalDipoles * 3))

        for m in range(self.nMeasurement):

            block = np.empty((1,self.totalDipoles * 3))

            x = self.xLocations[m]
            y = self.yLocations[m]
            z = self.zLocations[m]

            for i in range(self.totalDipoles):

                a = self.dipole[i].x
                b = self.dipole[i].y
                c = self.dipole[i].z

                mx = (3*(a-x)*(b-y)) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))
                my = (-1*(a-x)**2+2*(b-y)**2-1*(c-z)**2) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))
                mz = (3*(b-y)*(c-z)) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))

                comp = np.hstack((mx,my,mz))

                block[0,i*3:i*3+3] = comp

            A[m,:] = block

        A *= 1e-7

        return_dict[procnum] = A


    def calculate_bz(self, procnum, return_dict):
        """
        Calculates the matrix for all bz measurements.
        Inputs:
         procnum: <int> process number, used by multiprocessing
         return_dict: <dictionary> shared variable between processes, contains the final matrices for Bx, By, Bz
        Outputs:
         None
        """

        A = np.zeros((self.nMeasurement, self.totalDipoles * 3))

        for m in range(self.nMeasurement):

            block = np.empty((1,self.totalDipoles * 3))

            x = self.xLocations[m]
            y = self.yLocations[m]
            z = self.zLocations[m]

            for i in range(self.totalDipoles):

                a = self.dipole[i].x
                b = self.dipole[i].y
                c = self.dipole[i].z

                mx = (3*(a-x)*(c-z)) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))
                my = (3*(b-y)*(c-z)) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))
                mz = (-1*(a-x)**2-(b-y)**2+2*(c-z)**2) / (((a-x)**2+(b-y)**2+(c-z)**2)**(5/2))

                comp = np.hstack((mx,my,mz))

                block[0,i*3:i*3+3] = comp

            A[m,:] = block

        A *= 1e-7

        return_dict[procnum] = A


    def box(self, start, end, nPoints1D, nPoints2D, nPoints3D):
        """
        Creates a box of locations for the test dipoles based on a starting and ending point.
        Inputs:
         start: <list[float]> starting point [x,y,z]
         end: <list[float]> ending point [x,y,z]
         nPoints1D: <int> number of test dipoles along the x axis
         nPoints2D: <int> number of test dipoles along the y axis
         nPoints3D: <int> number of test dipoles along the z axis
        Outputs:
         None
        """

        self.totalDipoles = nPoints1D * nPoints2D * nPoints3D
        xDivisions = np.linspace(start[0], end[0], nPoints1D)
        yDivisions = np.linspace(start[1], end[1], nPoints2D)
        zDivisions = np.linspace(start[2], end[2], nPoints3D)

        self.X, self.Y, self.Z = np.meshgrid(xDivisions, yDivisions, zDivisions, indexing='ij')

        tempDipole = np.empty((nPoints1D, nPoints2D, nPoints3D), dtype=Dipole)
        for i,j,k in product(range(nPoints1D), range(nPoints2D), range(nPoints3D)):
            tempDipole[i,j,k] = Dipole(self.X[i,j,k],self.Y[i,j,k],self.Z[i,j,k])

        # 1D array
        self.dipole = np.reshape(tempDipole, (self.totalDipoles), order='C')
        

    def run(self, contour=False, density=False, vector=False):
        """
        Runs the algorithm on the dipoles, updates their dipole moment components and magnitudes.
        Inputs:
         contour: <bool> if true, the final visual is a contour plot
         density: <bool> if true, the final visual is a density plot
        Outputs:
         Plots.
        """

        start = time.time()

        # multiprocessing
        manager = mp.Manager()
        return_dict = manager.dict()
        jobs = []
        jobs.append(mp.Process(target=self.calculate_bx, args=(0, return_dict)))
        jobs.append(mp.Process(target=self.calculate_by, args=(1, return_dict)))
        jobs.append(mp.Process(target=self.calculate_bz, args=(2, return_dict)))

        # start the jobs
        for proc in jobs:
            proc.start()

        # wait for the jobs to finish
        for proc in jobs:
            proc.join()
        
        # get the matrices
        matrices = return_dict.values()

        # append the arrays
        A = np.vstack((matrices[0], matrices[1]))
        A = np.vstack((A, matrices[2]))

        # perform the matrix multiplication
        aPlus = np.linalg.pinv(A)
        self.dipoleComponents = np.matmul(aPlus, self.magVector)

        end = time.time() - start
        print(f"Computation Time: {end} s")

        # assign the components to the dipoles
        for i, ele in enumerate(self.dipole):
            ele.mx = self.dipoleComponents[i*3 + 0]
            ele.my = self.dipoleComponents[i*3 + 1]
            ele.mz = self.dipoleComponents[i*3 + 2]

            ele.mag = (ele.mx**2+ele.my**2+ele.mz**2)**(1/2)

        # # print out the magnitudes
        # for i, ele in enumerate(self.dipole):
        #     print(f"Dipole {i+1}:\nmoment: ({ele.mx}, {ele.my}, {ele.mz}) \nlocation: ({ele.x}, {ele.y}, {ele.z})\nMagnitude: {ele.mag}\n")

        if contour:
            self.plotContour()
        elif density:
            self.plotDensity()
        elif vector:
            self.plotVector()
        else:
            print("Please specify a visual results option in run()!")


    def plotDensity(self):
        """
        Plots a 3D plot of points with the size dependent on dipole magnitude. Deletes dipoles that are
        extremely wrong.
        Inputs:
         None
        Outputs:
         3D plot in mayaVI.
        """

        xplot   = np.empty((self.totalDipoles,))
        yplot   = np.empty((self.totalDipoles,))
        zplot   = np.empty((self.totalDipoles,))
        magplot = np.empty((self.totalDipoles,))

        # get the location and magnitude of each dipole
        indices = []
        for i, ele in enumerate(self.dipole):
            xplot[i] = ele.x
            yplot[i] = ele.y
            zplot[i] = ele.z
            magplot[i] = ele.mag

            # tolerance correction
            if ele.mag < 0.5e-3:
                indices.append(i)
            if ele.mag > 0.5e1:
                indices.append(i)

        indices = np.array(indices, dtype=int)

        # delete dipoles that are really wrong
        self.dipole = np.delete(self.dipole, indices)

        if self.dipole.shape[0] == 0:
            raise RuntimeError ("All dipoles are out of range!")

        mlab.points3d(xplot,yplot,zplot,magplot)
        mlab.axes(extent=[-0.4, 0.4, -0.4, 0.4, -0.4, 0.4])
        mlab.show()


    def plotVector(self):
        """
        Plots a 3D plot of vectors of the calculated dipole moment. Deletes dipoles that are
        extremely wrong.
        Inputs:
         None
        Outputs:
         3D plot in mayaVI.
        """

        xplot   = np.empty((self.totalDipoles,))
        yplot   = np.empty((self.totalDipoles,))
        zplot   = np.empty((self.totalDipoles,))
        magplot = np.empty((self.totalDipoles,))
        mx = np.empty((self.totalDipoles,))
        my = np.empty((self.totalDipoles,))
        mz = np.empty((self.totalDipoles,))

        # get the location and magnitude of each dipole
        indices = []
        for i, ele in enumerate(self.dipole):
            xplot[i] = ele.x
            yplot[i] = ele.y
            zplot[i] = ele.z
            mx[i] = ele.mx
            my[i] = ele.my
            mz[i] = ele.mz
            magplot[i] = ele.mag

            # tolerance correction
            if ele.mag < 0.5e-3:
                indices.append(i)
            if ele.mag > 0.5e1:
                indices.append(i)

        indices = np.array(indices, dtype=int)

        # delete dipoles that are really wrong
        self.dipole = np.delete(self.dipole, indices)

        if self.dipole.shape[0] == 0:
            raise RuntimeError ("All dipoles are out of range!")

        mlab.quiver3d(xplot, yplot, zplot, mx, my, mz, colormap='gist_rainbow')
        mlab.axes(extent=[-0.3, 0.3, -0.3, 0.3, -0.3, 0.3])
        mlab.show()


    def plotContour(self):
        """
        Plots a XY contour plot at each vertical position Z in the box of test dipoles.
        Inputs:
         None
        Outputs:
         Multiple contour plots in matplotlib
        """

        # create a contour plot at each vertical position
        nPoints3D = self.Z.shape[2]
        for k in range(nPoints3D):

            # get the 2D array from the 3D array
            X = self.X[:,:,k]
            Y = self.Y[:,:,k]

            nPoints1D = X.shape[0]
            nPoints2D = Y.shape[1]

            # create a 2D array of magnitude values for each location
            M = np.empty((nPoints1D, nPoints2D))
            for i in range(nPoints1D):
                for j in range(nPoints2D):
                    M[i,j] = self.dipole[i*nPoints1D*nPoints1D + j*nPoints2D + k].mag

            contLevels = np.linspace(0, 2, 100)
            plt.contour(X,Y,M, levels=contLevels)
            plt.title(f'Height (Z) = {self.Z[0,0,k]} m')
            plt.show()


if __name__ == '__main__':

    m = MultipleDipoles(r"Data\COMSOL Quad R=0.1 m LESS.txt")
    m.box([-0.1,-0.1,-0.1],[0.1,0.1,0.1],10,10,10)
    m.run(vector=True)