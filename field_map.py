from auto_map import AutoMap
from Assets.nscg3 import NSCG3
from Assets.tfm1186 import TFM1186

import numpy as np
import pandas as pd 

class FieldMap(AutoMap):
    """
    Sub class of AutoMap, used for mapping routines that do not involve rotations.
    Stores data for the background field map and the object field map.
    """

    def __init__(self,simulate=False):
        super().__init__(simulate=simulate)
        resources = TFM1186.GetResources()
        self.tfm = TFM1186(resources[0],simulate=simulate)
        # data for a completed background map
        self.backgroundData = None
        # data for a completed object map
        self.objectData = None
        self.main_filename = None
        self.numPoints = self.points.shape[1]
        # placeholder array for measured data
        self.data = np.empty((self.numPoints, 6))

    def trigger(self,index,**kwargs):
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
        # self.data.iloc[index,:] = np.hstack((self.points[:,index].transpose(),B))
        self.data[index,:] = np.hstack((self.points[:, index].transpose(), B))

    
    def processData(self, type):
        """
        Transforms the data into the lab frame and stores it.
        Stores as either a background map or an object map
        """

        # reflect the z coordinate
        self.data[:,0] *= -1
        # reflect the by component
        self.data[:4] *= -1
 
        if type=="background":
            self.backgroundData = self.data
        elif type=="object":
            self.objectData = self.data
        else:
            print("Type of map (background / object) not defined.")


    def mapStarted(self,**kwargs):
        pass
        # numPoints = self.points.shape[1]
        # self.data = pd.DataFrame(np.zeros((numPoints,6)),columns=['X','Y','Z','Bx','By','Bz'])
        # self.data = np.empty((numPoints, 6))
        # self.main_filename = 'Data.csv'
    
    def mapEnded(self,**kwargs):
        pass
        # print("Map finished.")
        # self.data.to_csv(self.main_filename)
        # self.processData()

    # def processData(self, **kwargs):
        # """
        # Transforms data to be in the lab frame.
        # """

        # # read the most recent data set
        # df = pd.read_csv(r"%s" %self.main_filename, index_col=0)

        # # transformations
        # x = df['X']
        # y = df['Y']
        # z = df['Z']
        # bx = df['Bz']
        # by = df['By']
        # bz = df['Bx']

        # z *= -1
        # by *= -1

        # # convert to numpy arrays
        # x = x.to_numpy()
        # y = y.to_numpy()
        # z = z.to_numpy()
        # bx = bx.to_numpy()
        # by = by.to_numpy()
        # bz = bz.to_numpy()

        # nPoints = len(x)

        # positionVector = np.vstack((x,y,z))
        # magneticVector = np.vstack((bx,by,bz))

        # finalData = pd.DataFrame(np.zeros((nPoints,6)),columns=['X','Y','Z','Bx','By','Bz'])

        # for i in range(nPoints):
            # finalData.iloc[i,:] = np.hstack((positionVector[:,i].transpose(), magneticVector[:,i].transpose()))

        # finalData.to_csv(self.main_filename)

