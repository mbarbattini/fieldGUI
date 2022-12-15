"""
A PyQt application for the 3-axis magnetic field mapping robot.
Created by Matthew Barbattini on 12/12/2022

Features:
1. Manually control the 3-axis robot with simple commands
2. Run automatic mappings of various shapes (plane, box, sphere)
3. Plot the measured field
4. Calculate the effective dipole moment of the object



"""
#TODO Add options for XY, YZ, XZ planes

from Assets.nscg3 import NSCG3
from Assets.tfm1186 import TFM1186
from field_map import FieldMap
from rotational_map import RotationalMap
from field_map_data_processing import FieldMapDataProcessing

# from PyQt6.QtWidgets import (QApplication, QGridLayout,QStackedLayout,QCheckBox,QFormLayout)
    # QHBoxLayout,
    # QLabel,
    # QPushButton,
    # QStatusBar,
    # QVBoxLayout,
    # QWidget,
    # QMainWindow, 
    # QToolBar,
    # QLineEdit,
    # QGridLayout,
    # QComboBox,
    # QRadioButton)

from PyQt5.QtWidgets import *

from PyQt6.QtCore import QSocketNotifier, Qt, QRect
import sys
import numpy as np

# to embed matplotlib plots in PyQt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

class FieldMapGUI(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Field Mapping v.1.0.0")
        # self.setFixedSize(900, 1000)
        # self.generalLayout = QGridLayout()
        self.generalLayout = QHBoxLayout()
        centralWidget = QWidget(self)
        centralWidget.setLayout(self.generalLayout)
        # mainHorizontalLayout = QHBoxLayout()
        # centralWidget.setLayout(mainHorizontalLayout)
        self.setCentralWidget(centralWidget)

        # default map type is plane
        self.mapType = 0
        # default map type is not rotational
        self.rotationalMapFlag = False
        # default background subtraction is False
        self.backgroundSubtractionFlag = False
        # the field mapping object, either FieldMap() or RotationalMap()
        self.mapObj= None
        # NSCG3 object for 3-axis robot
        self.threeAxisRobot = NSCG3("169.254.154.15", "XYZ", simulate=True)
        # NSCG3 object for rotational robot
        self.rotationalRobot = NSCG3("192.169.20.99", "XYZ", simulate=True)
        self.parametersSet = False
        # an object that calculates the effective dipole moment
        self.dataProcessing = FieldMapDataProcessing()
        self.mappingComplete = False 
        self.corner1Coord = np.array([0.0, 0.0, 0.0])
        self.corner2Coord = np.array([0.0, 0.0, 0.0])

        # global widgets
        self.coordinateWindow = None
        self.stackedLayoutCoord = None
        self.mapTypeComboBox = None
        self.movementValueInput = None

        # menu bar at the top of the window
        self.menu = self.menuBar().addMenu("&Menu")
        self.menu.addAction("&Exit", self.close)

        # status bar at the bottom of the window
        self.status = QStatusBar()
        self.status.showMessage("I'm the Status Bar")
        self.setStatusBar(self.status)

        # create each of the horizontal groups of the GUI, from left to right
        self.createGroup1Buttons()
        self.createGroup2Buttons()
        self.createGroup3Buttons()


    def createGroup1Buttons(self):
        """
        Creates the general purpose buttons on the left most horizontal pane 
        """
        
        def btnState(button):
            if button.text() == "Subtract Background":
                self.subtractBackgroundFlag = True if button.isChecked() else False
            elif button.text() == "Rotational":
                self.rotationalMapFlag = True if button.isChecked() else False

        # create the stacked layout for column 1
        group1Layout = QVBoxLayout()
        self.generalLayout.addLayout(group1Layout)

        # create the start map button
        startMapButton = QPushButton("Start Mapping")
        startMapButton.clicked.connect(self.startMapping)
        group1Layout.addWidget(startMapButton)

        # create the background subtract checkbox
        backSubButton = QCheckBox("Subtract Background")
        backSubButton.stateChanged.connect(lambda: btnState(backSubButton))
        group1Layout.addWidget(backSubButton)

        
        # create the calculate dipole moment button
        calcDMButton = QPushButton("Calculate Dipole Moment")
        calcDMButton.clicked.connect(self.calculateDipoleMoment)
        group1Layout.addWidget(calcDMButton)

    def createGroup2Buttons(self):
        """
        Creates the robot movement buttons in the middle pane
        """
        group2Layout = QGridLayout()
        self.generalLayout.addLayout(group2Layout)

        self.plusXButton = QPushButton("+X", self)
        # self.plusXButton.resize(150, 150)
        minusXButton = QPushButton("-X")
        plusYButton = QPushButton("+Y")
        minusYButton = QPushButton("-Y")
        plusZButton = QPushButton("+Z")
        minusZButton = QPushButton("-Z")
        zeroButton = QPushButton("Zero Position")
        setCorner1Button = QPushButton("Set Corner 1")
        setCorner2Button = QPushButton("Set Corner 2")
        self.movementValueInput = QLineEdit()
        self.movementValueInput.setText("10")
        self.distanceClickLabel = QLabel()
        self.distanceClickLabel.setText("Distance Per Click (mm)")

        self.coordinateWindow = QLineEdit()
        self.coordinateWindow.setReadOnly(True)
        self.coordinateWindow.setFixedHeight(25)
        # self.coordinateWindow.setFixedWidth(240)
        # get the current position of the robot and set the text in the coordinate window
        currentPosition = self.threeAxisRobot.position
        self.coordinateWindow.setText(f"{currentPosition[0]:.3f}, {currentPosition[1]:.3f}, {currentPosition[2]:.3f}")
        coordinateWindowLabel = QLabel()
        coordinateWindowLabel.setText("Current Position (m):")

        # connect the buttons to functions 
        self.plusXButton.clicked.connect(self.moveXplus)
        minusXButton.clicked.connect(self.moveXminus)
        plusYButton.clicked.connect(self.moveYplus)
        minusYButton.clicked.connect(self.moveYminus)
        plusZButton.clicked.connect(self.moveZplus)
        minusZButton.clicked.connect(self.moveZminus)
        zeroButton.clicked.connect(self.zeroPosition)
        setCorner1Button.clicked.connect(self.setCorner1)
        setCorner2Button.clicked.connect(self.setCorner2)

        group2Layout.addWidget(self.plusXButton, 1, 0)
        group2Layout.addWidget(minusXButton, 2, 0)
        group2Layout.addWidget(plusYButton, 1, 1)
        group2Layout.addWidget(minusYButton, 2, 1)
        group2Layout.addWidget(plusZButton, 1, 2)
        group2Layout.addWidget(minusZButton, 2, 2)
        group2Layout.addWidget(zeroButton, 3, 1)
        group2Layout.addWidget(setCorner1Button, 3, 0)
        group2Layout.addWidget(setCorner2Button, 3, 2)
        group2Layout.addWidget(coordinateWindowLabel, 0, 0)
        group2Layout.addWidget(self.coordinateWindow, 0, 1)
        group2Layout.addWidget(self.movementValueInput, 4, 1)
        group2Layout.addWidget(self.distanceClickLabel, 4, 0)



    def createGroup3Buttons(self):
        """
        Creates the panes that show coordinate parameters on the right most horizontal pane
        """
        self.group3VerticalLayout = QVBoxLayout()
        self.generalLayout.addLayout(self.group3VerticalLayout)

        # create the map type dropdown
        self.mapTypeComboBox = QComboBox()
        self.mapTypeComboBox.addItems(["XY Plane", "Box", "Sphere"])
        self.mapTypeComboBox.currentIndexChanged.connect(self.mapTypeChange) 
        self.group3VerticalLayout.addWidget(self.mapTypeComboBox)

        self.stackedLayoutCoord = QStackedLayout()
        self.group3VerticalLayout.addLayout(self.stackedLayoutCoord)
        # self.generalLayout.addLayout(self.stackedLayoutCoord)
        pagePlane = QWidget()
        pageLayoutPlane = QFormLayout()

        # xy plane page
        self.corner1PlaneLineEdit = QLineEdit()
        self.corner2PlaneLineEdit = QLineEdit()
        self.heightZPlaneLineEdit = QLineEdit()
        self.heightZPlaneLineEdit.setReadOnly(True)
        self.nPointsXPlaneSpinBox = QSpinBox()
        self.nPointsXPlaneSpinBox.setMinimum(2)
        self.nPointsYPlaneSpinBox = QSpinBox()
        self.nPointsYPlaneSpinBox.setMinimum(2)

    
        pageLayoutPlane.addRow("First Corner Position", self.corner1PlaneLineEdit)
        pageLayoutPlane.addRow("Second Corner Position", self.corner2PlaneLineEdit)
        pageLayoutPlane.addRow("Number Points X", self.nPointsXPlaneSpinBox)
        pageLayoutPlane.addRow("Number Points Y", self.nPointsYPlaneSpinBox)
        pageLayoutPlane.addRow("Height Z (m)", self.heightZPlaneLineEdit)

        pagePlane.setLayout(pageLayoutPlane)
        self.stackedLayoutCoord.addWidget(pagePlane)

        # box page
        pageBox = QWidget()
        pageLayoutBox = QFormLayout() 

        self.corner1BoxLineEdit = QLineEdit()
        self.corner2BoxLineEdit = QLineEdit()
        self.nPointsXBoxSpinBox= QSpinBox()
        self.nPointsYBoxSpinBox = QSpinBox()
        self.nPointsZBoxSpinBox = QSpinBox()
        self.nPointsXBoxSpinBox.setMinimum(2)
        self.nPointsYBoxSpinBox.setMinimum(2)
        self.nPointsZBoxSpinBox.setMinimum(2)
        pageLayoutBox.addRow("First Corner Position", self.corner1BoxLineEdit)
        pageLayoutBox.addRow("Second Corner Position", self.corner2BoxLineEdit)
        pageLayoutBox.addRow("Number Points X", self.nPointsXBoxSpinBox)
        pageLayoutBox.addRow("Number Points Y", self.nPointsYBoxSpinBox)
        pageLayoutBox.addRow("Number Points Z", self.nPointsZBoxSpinBox)
        pageBox.setLayout(pageLayoutBox)
        self.stackedLayoutCoord.addWidget(pageBox)

        # sphere page
        pageSphere = QWidget()
        pageLayoutSphere = QFormLayout()
        
        self.radiusSphereLineEdit = QLineEdit()
        self.densitySphereSpinBox = QSpinBox()
        self.densitySphereSpinBox.setMinimum(10)
        self.densitySphereSpinBox.setMaximum(10000)
        self.minHeightSphereLineEdit = QLineEdit()
        pageLayoutSphere.addRow("Radius (mm)", self.radiusSphereLineEdit)
        pageLayoutSphere.addRow("Number of Points", self.densitySphereSpinBox)
        pageLayoutSphere.addRow("Minimum Z Height (mm)", self.minHeightSphereLineEdit)

        pageSphere.setLayout(pageLayoutSphere)
        self.stackedLayoutCoord.addWidget(pageSphere)


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
                self.mapObj.rotationalPlane(6, self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY)
            elif self.mapType == 1:
                self.mapObj.rotationalBox(6, self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY, self.nPointsZ)
            elif self.mapType == 2:
                self.mapObj.rotationalSphere(6, self.radiusSphere, self.nPointsSphere, self.minHeightSphere)
        # normal map
        else:
            if self.mapType == 0:
                self.mapObj.plane(self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY)
            elif self.mapType == 1:
                self.mapObj.box(self.corner1Coord, self.corner2Coord, self.nPointsX, self.nPointsY, self.nPointsZ)
            elif self.mapType == 2:
                self.mapObj.sphere(self.radiusSphere, self.nPointsSphere, self.minHeightSphere)

    def getDistancePerClick(self):
        """
        Reads the input window for the distance per click in mm.
        Returns the value in meters.
        """
        rawString = self.movementValueInput.text()
        rawFloat = 0.0
        try:
            rawFloat = float(rawString)
        except ValueError:
            print("Please enter a valid number for the movement distance!")
        return abs(rawFloat / 1000)



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
        self.heightZPlaneLineEdit.setText(f"{self.corner2Coord[2]:.3f}")

    def mapTypeChange(self, i):
        """
        Updates the type of mapping geometry
        0: Plane, 1: Box, 2: Sphere 
        """
        # print(f"current index: {i}")
        self.mapType = i
        # switch the coordinate input window
        self.stackedLayoutCoord.setCurrentIndex(self.mapTypeComboBox.currentIndex())

    def clearDisplayValues(self):
        """
        Resets all parameters
        """
        self.coordinateWindow.setText("0.000, 0.000, 0.000")
        self.corner1BoxLineEdit.clear()
        self.corner1PlaneLineEdit.clear()
        self.corner2BoxLineEdit.clear()
        self.corner2PlaneLineEdit.clear()
        self.heightZPlaneLineEdit.clear()


    def updateParameters(self):
        """
        Updates the parameters for the selected mapping scheme.
        Returns true if all parameters are set, false if not.
        """
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
                print("Please enter a valid set of 3 coordinates separated by a \",\"")
                return False
            # if user typed in coordinates, make sure there are 3 and only 3
            if len(self.corner1Coord) != 3 or len(self.corner2Coord) != 3:
                print("Please enter exactly 3 coordinates.")
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
                print("Please enter a valid set of 3 coordinates separated by a \",\"")
                return False
            # if user typed in coordinates, make sure there are 3 and only 3
            if len(self.corner1Coord) != 3 or len(self.corner2Coord) != 3:
                print("Please enter exactly 3 coordinates.")
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

    def startMapping(self):
        """
        Begins the mapping scheme.
        """

        # if the user has not entered parameters for mapping scheme, return
        if not self.updateParameters():
            print("Parameters are not set for the mapping scheme!")
            return

        # create the mapping object
        self.implementMappingObject(_simulate=False)


        if self.backgroundSubtractionFlag:
            readyInput = input("Press the \"Ready\" button when the object is removed from the apparatus so mapping can begin...")
            print("Beginning to map the background field...")
            self.mapObj.map()
            self.mapObj.processData(type="background")
            print("Background map has ended. Please insert the object to be mapped.")
            print("Press the \"Ready\" button when the object is ready to be mapped.")
            #TODO Wait for the button to be pressed to execute the rest of the code
            self.mapObj.map()
            self.mapObj.processData(type="object")
            print("Mapping is complete.")
            self.mappingComplete = True
        else:
            readyInput = input("Press the \"Ready\" button when the object is ready to be mapped.")
            self.mapObj.map()
            self.mapObj.processData(type="object")
            print("Mapping is complete.")
            self.mappingComplete = True

    #TODO add a dropdown for the type of algorithm
    def calculateDipoleMoment(self, method="lm"):
        if not self.mappingComplete:
            print("Mapping is not complete!")
            return

        if method=="lm":
            self.dataProcessing.lm()
        elif method=="lmfit":
            self.dataProcessing.lmfit()

def main():
    fieldMapApp = QApplication([])
    fieldMap = FieldMapGUI()
    fieldMap.show()
    sys.exit(fieldMapApp.exec())



if __name__ == "__main__":
    main()
