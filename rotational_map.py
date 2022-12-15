from auto_map import AutoMap
from Assets.nscg3 import NSCG3
from Assets.tfm1186 import TFM1186

import numpy as np
import math
import pandas as pd

class RotationalMap(AutoMap):
    """
    Sub class of AutoMap, used to perform automatic mapping routines including rotations with the rotational robot.
    Member variables:
     self.rotationalNewmark: <NSCG3> object of the rotational newmark controller
     self.tfm: <TFM1186> object of the adapter for the Metrolab magnetometer, from TFM1186 file
     self.data: <pd.Dataframe> pandas dataframe where data is stored
     self.temp_filename: <string> name of the .csv file where data is stored for each individual map
     self.main_filename: <string> name of the .csv file where final data is stored, includes all rotations
    """

    def __init__(self, addr='192.168.20.99',simulate=False):
        """
        Creates a new NSCG3 object for the rotational newmark controller.
        """
        super().__init__(simulate=simulate)
        self.rotationalNewmark = NSCG3(addr,'X',simulate=simulate)
        resources = TFM1186.GetResources()
        self.tfm = TFM1186(resources[0],simulate=simulate)
        self.data = None
        self.temp_filename = None
        self.main_filename = None




    def trigger(self,index,**kwargs):
        """
        Arbitrary method called during each visit to a point. In this case, data is read from the magnetometer
        and stored in the data member variable.
        Inputs:
         Index: <int> index of the set of points 
        Outputs:
         None
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
        self.data.iloc[index,:] = np.hstack((self.points[:,index].transpose(),B))





    def mapStarted(self,**kwargs):
        """
        Creates the dataframe for measurements to be stored.
        """
        numPoints = self.points.shape[1]
        self.data = pd.DataFrame(np.zeros((numPoints,6)),columns=['X','Y','Z','Bx','By','Bz'])
        self.temp_filename = 'Data.csv'





    def mapEnded(self,**kwargs):
        """
        Saves data to a csv file with the name of the temp_filename member variable.
        """
        self.data.to_csv(self.temp_filename)

    



    def rotate(self, nRotations,speed=30,acceleration=20,deceleration=20,**kwargs):
        
        """
        Rotates the axis of the rotational newmark controller a certain angle based on the number
        of desired rotations.
        Inputs:
         nRotations: <int> number of rotations
         speed: <int> newmark speed
         acceleration: <int> newmark acceleration
         deceleration: <int> newmark deceleration
        Outputs:
         None
        """
        # this value is the range of the rotational newmark positioner to make one revolution. 
        rotationalRange1Revolution = 575
        distance = [-rotationalRange1Revolution / nRotations]

        self.rotationalNewmark.speed = speed
        self.rotationalNewmark.acceleration = acceleration
        self.rotationalNewmark.deceleration = deceleration

        self.rotationalNewmark.moveRelative(distance)
        self.rotationalNewmark.wait()





    def rotationX(self, _angle, **kwargs):
        """
        Defines a 3D rotation matrix about the x axis.
        Inputs:
         _angle: angle in degrees
        Outputs:
         (3,3) np.array rotational matrix
        """
        angle = math.radians(_angle)
        return np.array(([1 ,       0       ,       0        ],
                        [0 , np.cos(angle)  , -np.sin(angle) ],
                        [0 , np.sin(angle) ,  np.cos(angle) ]), dtype=float)





    def rotationY(self, _angle, **kwargs):
        """
        Defines a 3D rotation matrix about the y axis.
        Inputs:
         _angle: angle in degrees
        Outputs:
         (3,3) np.array rotational matrix
        """
        angle = math.radians(_angle)
        return np.array(([np.cos(angle) , 0 , np.sin(angle)],
                        [0              , 1 ,             0],
                        [-np.sin(angle) , 0 , np.cos(angle)]), dtype=float)





    def rotationZ(self, _angle):
        """
        Defines a 3D rotation matrix about the z axis.
        Inputs:
         _angle: angle in degrees
        Outputs:
         (3,3) np.array rotational matrix
        """
        angle = math.radians(_angle)
        return np.array(([np.cos(angle), -np.sin(angle) , 0],
                        [np.sin(angle) , np.cos(angle)  , 0],
                        [       0      ,        0       , 1]), dtype=float)





    def processDataBackgroundSubtract(self, rotationNumber, totalRotations, main_filename, **kwargs):
        """
        Processes data when the object is rotated a certain angle specified by the user, subtracts the background. 
        Saves transformed data to the master file.
        Inputs:
         rotationNumber: <int> the current rotation
         totalRotations: <int> the total number of rotations
         main_filename: <string> the final file where transformed data will be saved
        Outputs:
         None
        """

        angle = 360 // totalRotations

        self.main_filename = main_filename

        # read the most recent data set
        df = pd.read_csv(r"%s" %self.temp_filename, index_col=0)

        # transformations
        x = df['X']
        y = df['Y']
        z = df['Z']
        bx = df['Bz']
        by = df['By']
        bz = df['Bx']

        z *= -1
        by *= -1

        # convert to numpy arrays
        x = x.to_numpy()
        y = y.to_numpy()
        z = z.to_numpy()
        bx = bx.to_numpy()
        by = by.to_numpy()
        bz = bz.to_numpy()

        nPoints = len(x)

        # stack into vectors
        positionVector = np.vstack((x,y,z))
        magneticVector = np.vstack((bx,by,bz))

        finalMagnetic = np.empty((3,nPoints))
        finalPositions = np.empty((3,nPoints))

        singleBackground = np.array([ 20880.00 ,
                                      -4478.68,
                                     -39532.40 ])

        # backgroundVector = np.vstack((backBx, backBy, backBz))
        backgroundVector = np.zeros((3, nPoints))
        for i in range(nPoints):
            backgroundVector[:,i] = np.array([ 20880.00,-4478.68,-39532.40 ])
        
        finalBackground = np.empty((3,nPoints))

        """
        Rotation transformations for spinning the object on the axis of the rotation newmark robot. 
        First measurement does not need to be rotated.
        """
        if rotationNumber != 0:
            for i in range(nPoints):

                tempVectorMagnetic = magneticVector[:,i]
                tempVectorMagnetic = np.matmul(self.rotationX((rotationNumber * angle)), tempVectorMagnetic)
                finalMagnetic[:,i] = tempVectorMagnetic

                tempVectorPosition = positionVector[:,i]
                tempVectorPosition = np.matmul(self.rotationX((rotationNumber * angle)), tempVectorPosition)
                finalPositions[:,i] = tempVectorPosition

                tempVectorBackground = singleBackground
                tempVectorBackground = np.matmul(self.rotationX((rotationNumber * angle)), tempVectorBackground)
                finalBackground[:,i] = tempVectorBackground
        else:
            finalMagnetic = magneticVector
            finalPositions = positionVector
            finalBackground = backgroundVector

        # subtract the background before saving
        for i in range(nPoints):
            finalMagnetic[:,i] = finalMagnetic[:,i] - finalBackground[:,i]

        # read the main data frame
        main = pd.read_csv(r"%s" %main_filename, index_col=0)

        # create the new dataframe
        transformedData = pd.DataFrame(np.zeros((nPoints,6)),columns=['X','Y','Z','Bx','By','Bz'])

        # add the data to the transformed dataframe
        for i in range(nPoints):
            transformedData.iloc[i,:] = np.hstack((finalPositions[:,i].transpose(), finalMagnetic[:,i].transpose()))

        # append transformed data frame to the main data frame
        frames = [main, transformedData]
        final = pd.concat(frames, ignore_index=True)

        # save the modified main data frame, overwrites existing
        final.to_csv(main_filename)




    def processData(self, rotationNumber, totalRotations, main_filename, **kwargs):
        """
        Processes data when the object is rotated a certain angle specified by the user. 
        Saves transformed data to the master file.
        Inputs:
         rotationNumber: <int> the current rotation
         totalRotations: <int> the total number of rotations
         main_filename: <string> name where the final data is stored
        Outputs:
         None
        """

        angle = 360 // totalRotations

        self.main_filename = main_filename

        # read the most recent data set
        df = pd.read_csv(r"%s" %self.temp_filename, index_col=0)

        # transformations
        x = df['X']
        y = df['Y']
        z = df['Z']
        bx = df['Bz']
        by = df['By']
        bz = df['Bx']

        z *= -1
        by *= -1

        # convert to numpy arrays
        x = x.to_numpy()
        y = y.to_numpy()
        z = z.to_numpy()
        bx = bx.to_numpy()
        by = by.to_numpy()
        bz = bz.to_numpy()

        nPoints = len(x)

        # stack into vectors
        positionVector = np.vstack((x,y,z))
        magneticVector = np.vstack((bx,by,bz))

        finalMagnetic = np.empty((3,nPoints))
        finalPositions = np.empty((3,nPoints))

        """
        Rotation transformations for spinning the object on the axis of the rotation newmark robot. 
        First measurement does not need to be rotated.
        """
        if rotationNumber != 0:
            for i in range(nPoints):
                tempVectorMagnetic = magneticVector[:,i]
                tempVectorMagnetic = np.matmul(self.rotationX((rotationNumber * angle)), tempVectorMagnetic)
                finalMagnetic[:,i] = tempVectorMagnetic
                tempVectorPosition = positionVector[:,i]
                tempVectorPosition = np.matmul(self.rotationX((rotationNumber * angle)), tempVectorPosition)
                finalPositions[:,i] = tempVectorPosition
        else:
            finalMagnetic = magneticVector
            finalPositions = positionVector

        # read the main data frame
        main = pd.read_csv(r"%s" %main_filename, index_col=0)

        # create the new dataframe
        transformedData = pd.DataFrame(np.zeros((nPoints,6)),columns=['X','Y','Z','Bx','By','Bz'])

        # add the data to the transformed dataframe
        for i in range(nPoints):
            transformedData.iloc[i,:] = np.hstack((finalPositions[:,i].transpose(), finalMagnetic[:,i].transpose()))

        # append transformed data frame to the main data frame
        frames = [main, transformedData]
        final = pd.concat(frames, ignore_index=True)

        # save the modified main data frame, overwrites existing
        final.to_csv(main_filename)




    def rotationalPlane(self, _nRotations, _start, _end, _nPoints1D, _nPoints2D, _main_filename, backgroundSubtract=False):
        """
        Automatic plane mapping with a specified number of rotations.
        Subtracts the background field with the key word argument set to True.
        Inputs:
        _nRotations: <int> number of rotations, aka total number of planes mapped
        _start: <int[]> [xStart, yStart, zStart] starting coordinates in terms of the 3 axis positioner
        _end: <int[]> [xEnd, yEnd, zEnd] ending coordinates in terms of the 3 axis positioner
        _nPoints1D <int> number of sample points along the first axis of the plane
        _nPoints2D <int> number of sample points along the second axis of the plane
        _main_filename <string> name of the .csv file where all the data will be stored, needs .csv extension
        Outputs:
         Vector plot of measured data in MayaVI
        """

        self.main_filename = _main_filename
        
        file = open(r"%s" %_main_filename, 'w')
        file.write(",X,Y,Z,Bx,By,Bz")
        file.close()

        for i in range(_nRotations):
            self.plane(_start, _end, _nPoints1D, _nPoints2D)
            self.optimizeTrajectory()
            self.map()
            self.rotate(_nRotations)
            
            if backgroundSubtract:
                self.processDataBackgroundSubtract(i, _nRotations, _main_filename)
            else:
                self.processData(i, _nRotations, _main_filename)

        self.visualize()
        self.rotationalNewmark.moveAbsolute([0])




    def rotationalBox(self, _nRotations,  _start, _end, _nPoints1D, _nPoints2D, _nPoints3D, _main_filename, backgroundSubtract=False):
        """
        Automatic mapping of a solid rectangular box with a specified number of points.
        Subtracts the background field with the key word argument set to True.
        Inputs:
         _nRotations: <int> number of rotations, aka total number of routines mapped
         _start: <int[]> [xStart, yStart, zStart] starting coordinates in terms of the 3 axis positioner
         _end: <int[]> [xEnd, yEnd, zEnd] ending coordinates in terms of the 3 axis positioner
         _nPoints1D: <int> number of sample points along the first axis of the plane
         _nPoints2D: <int> number of sample points along the second axis of the plane
         _nPoints3D: <int> number of sample points along the third axis of the plane
         _main_filename: <string> name of the .csv file where all the data will be stored, needs .csv extension
        Outputs:
         Vector plot of measured data in MayaVI
        """

        self.main_filename = _main_filename

        file = open(r"%s" %_main_filename, 'w')
        file.write(",X,Y,Z,Bx,By,Bz")
        file.close()

        for i in range(_nRotations):
            self.box(_start, _end, _nPoints1D, _nPoints2D,_nPoints3D)
            self.optimizeTrajectory()
            self.map()
            self.rotate(_nRotations)

            if backgroundSubtract:
                self.processDataBackgroundSubtract(i, _nRotations, _main_filename)
            else:
                self.processData(i, _nRotations, _main_filename)

        self.visualize()
        self.rotationalNewmark.moveAbsolute([0])




    def rotationalSphere(self, _nRotations, _radius, _nPoints, _minHeight, _main_filename, backgroundSubtract=False):
        """
        Maps out a equally-distributed top sphere above the object. Number of points is not the true value because some are
        cut out in the middle. Subtracts the background field with the key word argument set to True.
        Inputs:
         _nRotations: <int> number of rotations
         _radius: <float> radius of the sphere measured from the zeroing point
         _nPoints: <int> number of points on the sphere, more than actual
         _minHeight: <float> minimum height the sphere will extend to. Corresponds to the slice to create a hat of the sphere.
         _main_filename: <string> name of the .csv file where all the data will be stored, needs .csv extension
        Outputs:
         None
        """

        self.main_filename = _main_filename

        file = open(r"%s" %_main_filename, 'w')
        file.write(",X,Y,Z,Bx,By,Bz")
        file.close()

        for i in range(_nRotations):
            self.sphere(_radius, _nPoints, _minHeight)
            self.optimizeTrajectory()
            self.map()
            self.rotate( _nRotations)

            if backgroundSubtract:
                self.processDataBackgroundSubtract(i, _nRotations, _main_filename)
            else:
                self.processData(i, _nRotations, _main_filename)
                
        self.visualize()
        self.rotationalNewmark.moveAbsolute([0])