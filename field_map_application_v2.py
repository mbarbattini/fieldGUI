from PyQt5 import uic 
from PyQt5.QtWidgets import QMainWindow, QApplication
import sys
import os
import numpy as np
from Assets.nscg3 import NSCG3
from field_map_data_processing import FieldMapDataProcessing
from rotational_map import RotationalMap
from field_map import FieldMap
import datetime


class FieldMapUI(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("FieldMapping/form.ui", self)
        self.setWindowTitle("Field Mapping v.1.0.0")
        self.generalDisplayLineEdit.setText("Welcome to Field Mapping v.1.0.0")
        # default map type is non-rotational
        self.rotationalMapFlag = False
        # default background subtraction is False
        self.backgroundSubtractionFlag = False
        # the field mapping object, either FieldMap() or RotationalMap()
        self.mapObj= None 
        # NSCG3 object for three axis robot
        self.threeAxisRobot = NSCG3("169.254.154.15", "XYZ", simulate=True)
        # NSCG3 object for rotational robot
        self.rotationalRobot = NSCG3("192.169.20.99", "XYZ", simulate=True)
        # class for doing calculations
        self.parametersSet = False
        self.dataProcessing = FieldMapDataProcessing()
        self.mappingComplete = False 
        self.corner1Coord = np.array([0.0, 0.0, 0.0])
        self.corner2Coord = np.array([0.0, 0.0, 0.0])
        # default map type is box
        self.mapType = 0
        # variable for waiting until the user hits the ready button
        self.ready = False
        # default number of rotations for a rotational map
        self.nRotations = 2

        # modify properties for imported QtWidgets
        self.generalDisplayLineEdit.setReadOnly(True)
        self.nPointsXPlaneSpinBox.setMinimum(2)
        self.nPointsXBoxSpinBox.setMinimum(2)
        self.nPointsYPlaneSpinBox.setMinimum(2)
        self.nPointsYBoxSpinBox.setMinimum(2)
        self.nPointsZBoxSpinBox.setMinimum(2)
        self.densitySphereSpinBox.setMinimum(10)
        self.nRotationsSpinBox.setMinimum(2)

        # display the current position on start
        currentPosition = self.threeAxisRobot.position
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

        # connect buttons to functions
        self.minusXButton.clicked.connect(self.moveXminus)
        self.plusXButton.clicked.connect(self.moveXplus)
        self.minusYButton.clicked.connect(self.moveYminus)
        self.plusYButton.clicked.connect(self.moveYplus)
        self.minusZButton.clicked.connect(self.moveZminus)
        self.plusZButton.clicked.connect(self.moveZplus)
        self.setCorner1Button.clicked.connect(self.setCorner1)
        self.setCorner2Button.clicked.connect(self.setCorner2)
        self.setZeroButton.clicked.connect(self.zeroPosition)
        self.startMappingButton.clicked.connect(self.startMapping)
        self.calcDipoleMomentButton.clicked.connect(self.calculateDipoleMoment)
        self.readyUserButton.clicked.connect(self.readyButtonPressed)
        self.rotationalComboBox.currentIndexChanged.connect(self.rotationalTypeChanged)
        self.mapTypeComboBox.currentIndexChanged.connect(self.mapTypeChange)
        self.saveDataButton.clicked.connect(self.saveData)


    def mapTypeChange(self, i):
        """
        Updates the type of mapping geometry
        0: Plane, 1: Box, 2: Sphere 
        """
        # print(f"current index: {i}")
        self.mapType = i
        # switch the coordinate input window
        self.stackedMapTypeWidget.setCurrentIndex(self.mapTypeComboBox.currentIndex())


    def rotationalTypeChanged(self, i):
        """
        Shows the option for the number of rotations
        """
        self.rotations = i
        self.rotationsStackedWidget.setCurrentIndex(self.rotationalComboBox.currentIndex())

    def getDistancePerClick(self):
        """
        Reads the input window for the distance per click in mm.
        Returns the value in meters.
        """
        rawString = self.distancePerClickLineEdit.text()
        rawFloat = 0.0
        try:
            rawFloat = float(rawString)
        except ValueError:
            self.generalDisplayLineEdit.setText("Please enter a valid number for the movement distance!")
        return abs(rawFloat)



    def moveXplus(self):
        """
        Moves the three axis robot in the positive x direction.
        """
        moveValue = self.getDistancePerClick()
        currentPosition = self.threeAxisRobot.position
        newX = currentPosition[0] + moveValue
        currentPosition[0] = newX
        moveArr = np.array([moveValue, 0, 0])
        self.threeAxisRobot.moveRelative(moveArr)
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def moveXminus(self):
        """
        Moves the three axis robot in the minus x direction.
        """
        moveValue = self.getDistancePerClick()
        currentPosition = self.threeAxisRobot.position
        newX = currentPosition[0] - moveValue
        currentPosition[0] = newX
        moveArr = np.array([-moveValue, 0, 0])
        self.threeAxisRobot.moveRelative(moveArr)
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def moveYplus(self):
        """
        Moves the three axis robot in the positive y direction.
        """
        moveValue = self.getDistancePerClick()
        currentPosition = self.threeAxisRobot.position
        newY = currentPosition[1] + moveValue
        currentPosition[1] = newY
        moveArr = np.array([0, moveValue, 0])
        self.threeAxisRobot.moveRelative(moveArr)
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def moveYminus(self):
        """
        Moves the three axis robot in the minus y direction.
        """
        moveValue = self.getDistancePerClick()
        currentPosition = self.threeAxisRobot.position
        newY = currentPosition[1] - moveValue
        currentPosition[1] = newY
        moveArr = np.array([0, -moveValue, 0])
        self.threeAxisRobot.moveRelative(moveArr)
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def moveZplus(self):
        """
        Moves the three axis robot in the positive z direction.
        """
        moveValue = self.getDistancePerClick()
        currentPosition = self.threeAxisRobot.position
        newZ = currentPosition[2] + moveValue
        currentPosition[2] = newZ
        moveArr = np.array([0, 0, moveValue])
        self.threeAxisRobot.moveRelative(moveArr)
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def moveZminus(self):
        """
        Moves the three axis robot in the minus z direction.
        """
        moveValue = self.getDistancePerClick()
        currentPosition = self.threeAxisRobot.position
        newZ = currentPosition[2] - moveValue
        currentPosition[2] = newZ
        moveArr = np.array([0, 0, -moveValue])
        self.threeAxisRobot.moveRelative(moveArr)
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def zeroPosition(self):
        """
        Zeros the three axis robot at the current position.
        """
        self.threeAxisRobot.setZero()
        self.clearDisplayValues()

    def setCorner1(self):
        """
        Sets the first corner for the plane or box mapping scheme.
        """
        currentPosition = self.threeAxisRobot.position
        self.corner1Coord = currentPosition
        self.corner1BoxLineEdit.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")
        self.corner1PlaneLineEdit.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")

    def setCorner2(self):
        """
        Sets the second corner for the plane or box mapping scheme
        """
        currentPosition = self.threeAxisRobot.position
        self.corner2Coord = currentPosition
        self.corner2BoxLineEdit.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")
        self.corner2PlaneLineEdit.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")
        # xy plane should have the same z coordinate, so it's ok to set the label with the second corner
        # self.heightZPlaneLineEdit.setText(f"{self.corner2Coord[2]:.3f}")

    def implementMappingObject(self, _simulate):
        """
        Implements the mapping object, either a rotational map or a normal map.
        Adds the mapping type and the parameters.
        """
        self.mapObj = RotationalMap(simulate=_simulate) if self.rotationalMapFlag else FieldMap(simulate=_simulate)
        
        # add the type of mapping scheme to the object
        # rotational map
        if self.rotationalMapFlag:
            if self.mapType == 0:
                self.mapObj.rotationalPlane(self.nRotations, self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY)
            elif self.mapType == 1:
                self.mapObj.rotationalBox(self.nRotations, self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY, self.nPointsZ)
            elif self.mapType == 2:
                self.mapObj.rotationalSphere(self.nRotations, self.radiusSphere, self.nPointsSphere, self.minHeightSphere)
        # normal map
        else:
            if self.mapType == 0:
                self.mapObj.plane(self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY)
            elif self.mapType == 1:
                self.mapObj.box(self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY, self.nPointsZ)
            elif self.mapType == 2:
                self.mapObj.sphere(self.radiusSphere, self.nPointsSphere, self.minHeightSphere)

    def clearDisplayValues(self):
        """
        Resets all displayed parameters
        """
        self.coordinateWindow.setText("0.000, 0.000, 0.000")
        self.corner1BoxLineEdit.clear()
        self.corner1PlaneLineEdit.clear()
        self.corner2BoxLineEdit.clear()
        self.corner2PlaneLineEdit.clear()
        # self.heightZPlaneLineEdit.clear()    

    def updateParameters(self):
        """
        Updates the parameters for the selected mapping scheme.
        Returns true if all parameters are set, false if not.
        """

        self.algorithmType = self.algorithmTypeComboBox.currentIndex()

        if self.mapType == 0:
            # get all the parameters
            rawInputCorner1 = self.corner1PlaneLineEdit.text()
            rawInputCorner2 = self.corner2PlaneLineEdit.text()
            self.nPointsX = self.nPointsXPlaneSpinBox.value()
            self.nPointsY = self.nPointsYPlaneSpinBox.value()

            # make sure there is valid input
            if not rawInputCorner1 or not rawInputCorner2:
                return False
            try:
                self.corner1Coord = np.array(rawInputCorner1.split(', '), dtype=float)
                self.corner2Coord = np.array(rawInputCorner2.split(', '), dtype=float)
            except ValueError:
                self.generalDisplayLineEdit.setText("Please enter a valid set of 3 coordinates separated by a \",\"")
                # print("Please enter a valid set of 3 coordinates separated by a \",\"")
                return False
            # if user typed in coordinates, make sure there are 3 and only 3
            if len(self.corner1Coord) != 3 or len(self.corner2Coord) != 3:
                self.generalDisplayLineEdit.setText("Please enter exactly 3 coordinates.")
                # print("Please enter exactly 3 coordinates.")
                return False
            return True

        elif self.mapType == 1:
            # get all the parameters
            rawInputCorner1 = self.corner1BoxLineEdit.text()
            rawInputCorner2 = self.corner2BoxLineEdit.text()
            self.nPointsX = self.nPointsXBoxSpinBox.value()
            self.nPointsY = self.nPointsYBoxSpinBox.value()
            self.nPointsZ = self.nPointsZBoxSpinBox.value()

            # make sure that there is valid input
            if not rawInputCorner1 or not rawInputCorner2:
                return False
            try:
                self.corner1Coord = np.array(rawInputCorner1.split(', '), dtype=float)
                self.corner2Coord = np.array(rawInputCorner2.split(', '), dtype=float)
            except ValueError:
                self.generalDisplayLineEdit.setText("Please enter a valid set of 3 coordinates separated by a \",\"")
                return False
            # if user typed in coordinates, make sure there are 3 and only 3
            if len(self.corner1Coord) != 3 or len(self.corner2Coord) != 3:
                self.generalDisplayLineEdit.setText("Please enter exactly 3 coordinates.")
                return False
            return True

        elif self.mapType == 2:
            # get all the parameters
            rawInputRadius = self.radiusSphereLineEdit.text()
            rawInputMinHeight = self.minHeightSphereLineEdit.text()
            self.nPointsSphere = self.densitySphereSpinBox.value()

            # make sure that there is input
            if not rawInputRadius or not rawInputMinHeight:
                return False

            # update parameters
            self.radiusSphere = float(rawInputRadius) / 1e3
            self.minHeightSphere = float(rawInputMinHeight) / 1e3
            return True    

    def readyButtonPressed(self):
        self.ready = True
    
    def startMapping(self):
        """
        Begins the mapping scheme.
        """

        # if the user has not entered parameters for mapping scheme, return
        if not self.updateParameters():
            self.generalDisplayLineEdit.setText("Parameters are not set for the mapping scheme!")
            return

        # clear any saved data
        self.dataProcessing.clearData()

        # create the mapping object
        self.implementMappingObject(_simulate=True)

        # store the coordinate data in the dataProcessing object
        self.dataProcessing.importData(self.mapObj.points, type="coordinate")

        if self.backgroundSubtractionFlag:
            self.generalDisplayLineEdit.setText("Press the \"Ready\" button when the object is removed from the apparatus so mapping can begin...")
            self.ready = False
            while not self.ready: pass
            self.generalDisplayLineEdit.setText("Beginning to map the background field...")
            self.mapObj.map(type="background")
            self.mapObj.processData(type="background")
            # store the measured data in the dataProcessing object
            self.dataProcessing.importData(self.mapObj.backgroundData, type="background")
            self.generalDisplayLineEdit.setText("Background map has ended. Please insert the object to be mapped.\nPress the \"Ready\" button when the object is ready to be mapped.")
            self.ready = False
            while not self.ready: pass
            self.mapObj.map(type="object")
            # process data with transformations for lab frame and rotations
            self.mapObj.processData(type="object")
            # store the measured data in the dataProcessing object
            self.dataProcessing.importData(self.mapObj.objectData, type="object")
            self.generalDisplayLineEdit.setText("Mapping is complete.")
            self.mappingComplete = True
        else:
            self.generalDisplayLineEdit.setText("Press the \"Ready\" button when the object is ready to be mapped.")
            self.ready = False
            while not self.ready: pass
            self.mapObj.map()
            self.mapObj.processData(type="object")
            self.generalDisplayLineEdit.setText("Mapping is complete.")
            self.mappingComplete = True    


    def calculateDipoleMoment(self):
        """
        Calculates the dipole moment of the most recent mapping procedure.
        Prints the results to the window.
        """
        if not self.mappingComplete:
            self.generalDisplayLineEdit.setText("There is currently no stored data.\nPlease perform a mapping scheme and try again.")
            return

        if self.algorithmType==0:
            results = self.dataProcessing.lm()
            self.generalDisplayLineEdit.setText(results)
        elif self.algorithmType==1:
            results = self.dataProcessing.lmfit()
            self.generalDispalyLineEdit.setText(results)
    
    def saveData(self):
        """
        Saves data of the most recent mapping scheme in the cwd/Data
        Saves background field separately from measured field.
        """
        if not self.mappingComplete:
            self.generalDisplayLineEdit.setText("The mapping routine has not been completed.\nPlease perform a mapping routine and try again.")
            return
        now = datetime.now()
        now = now.strftime("%d/%m/%Y_%H:%M:%S")

        currentDirectory = os.getcwd()
        if self.backgroundSubtractionFlag:
            self.dataProcessing.exportDataBackground(
                self.mapObj.objectData,
                self.mapObj.backgroundData,
                now
            )
        else:
            self.dataProcess.exportDataNoBackground(
                self.mapObj.objectData,
                now
            )
            filenameObject = f"{currentDirectory}\\Data\\{now}_object.csv"
            filenameBackground = f"{currentDirectory}\\Data\\{now}_background.csv"
            os.makedirs(os.path.dirname(filenameObject), exist_ok=True) 
            os.makedirs(os.path.dirname(filenameBackground), exist_ok=True)
            self.dataProcessing.exportData(filenameBackground, type="background")
            self.dataProcessing.exportData(filenameObject, type="object")
        else:
            filenameObject = f"{currentDirectory}\\Data\\{now}_object.csv"
            os.makedirs(os.path.dirname(filenameObject), exist_ok=True) 
            self.dataProcessing.exportData(filenameObject, type="object")
        

def main():
    app = QApplication([])
    window = FieldMapUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()