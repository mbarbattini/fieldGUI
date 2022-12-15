# -*- coding: utf-8 -*-
"""
Created on Fri Jan 24 12:52:23 2020

@author: H392547
"""

from PyQt5 import QtWidgets, uic, QtCore, QtGui
import numpy as np
import sys
import os
from functools import reduce
[sys.path.append(i) for i in['.','..']]
l=[]
script_path=os.path.split(sys.argv[0])
for i in range(len(script_path)):
    sys.path.append( reduce(os.path.join,script_path[:i+1]))
from Assets.nscg3 import NSCG3
import time

n = NSCG3('169.254.154.15','XYZ',simulate=False)
# n = NSCG3('192.168.20.99','XYZ',simulate=False)
class UI(QtWidgets.QMainWindow):
    def __init__(self):
        
        super(UI, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi('RoboDriver.ui', self) # Load the .ui file
        self.bt1.clicked.connect(self.UP)
        self.bt2.clicked.connect(self.DOWN)
        self.bt3.clicked.connect(self.LEFT)
        self.bt4.clicked.connect(self.RIGHT)
        self.bt5.clicked.connect(self.AWAY)
        self.bt6.clicked.connect(self.TOWARD)
        self.bt7.clicked.connect(self.POSITION)
        self.bt8.clicked.connect(self.SETZERO)
        self.Line.clicked.connect(self.LINE)
        self.Cylinder.clicked.connect(self.CYLINDER)
        self.Box.clicked.connect(self.BOX)
        self.Sphere.clicked.connect(self.SPHERE)
        self.lineEdit.setText("10") 
        self.X_1.setText("0")
        self.Y_1.setText("0")
        self.Z_1.setText("0")
        self.X_3.setText("0")
        self.Y_3.setText("0")
        self.Z_3.setText("0")
        self.X_5.setText("0")
        self.Y_5.setText("0")
        self.Z_5.setText("0")
        self.Exit.clicked.connect(self.close)        
        self.POSITION()
        
    def SPHERE(self):
        radius=self.Radius_1.value()
        L=11
        radius=radius*(1/1000)
        from spherical import Spherical as sp
        quad=sp.sphericalHarmonicQuadrature(L,radius)
        x=quad.x
        y=quad.y
        z=quad.z
        points=np.vstack((x,y,z))
        points=points*1000
        from auto_map import FieldMap
        f=FieldMap()
        f.points=points
        f.optimizeTrajectory()
        f.map()
        
    def LINE(self):
        from auto_map import FieldMap
        try:
            f=FieldMap()
            x_1=self.X_1.text()
            y_1=self.Y_1.text()
            z_1=self.Z_1.text()
            x_2=self.X_2.text()
            y_2=self.Y_2.text()
            z_2=self.Z_2.text()
            x_1=float(x_1)
            y_1=float(y_1)
            z_1=float(z_1)
            x_2=float(x_2)
            y_2=float(y_2)
            z_2=float(z_2)
            points=self.NumPoints.value()
            f.line([x_1,y_1,z_1],[x_2,y_2,z_2],points)
            f.map()
            n.moveAbsolute([0,0,0])
        except:
            print("error")
            return
    
    def CYLINDER(self):
        try:
            from auto_map import FieldMap
            f=FieldMap()
            x_3=self.X_3.text()
            y_3=self.Y_3.text()
            z_3=self.Z_3.text()
            x_4=self.X_4.text()
            y_4=self.Y_4.text()
            z_4=self.Z_4.text()
            x_3=float(x_3)
            y_3=float(y_3)
            z_3=float(z_3)
            x_4=float(x_4)
            y_4=float(y_4)
            z_4=float(z_4)
            z_points=self.Z_Points.value()
            phi_points=self.Phi_Points.value()
            radius=self.Radius.value()
            f.cylinder([x_3,y_3,z_3],[x_4,y_4,z_4],radius,z_points,phi_points)
            f.optimizeTrajectory()
            f.map()
            n.moveAbsolute([0,0,0])
        except:
            print("error")
            return
    def BOX(self):
        try: 
            from auto_map import FieldMap
            f=FieldMap()
            x_5=self.X_5.text()
            y_5=self.Y_5.text()
            z_5=self.Z_5.text()
            x_6=self.X_6.text()
            y_6=self.Y_6.text()
            z_6=self.Z_6.text()
            x_5=float(x_5)
            y_5=float(y_5)
            z_5=float(z_5)
            x_6=float(x_6)
            y_6=float(y_6)
            z_6=float(z_6)
            x_points=self.x_points.value()
            y_points=self.y_points.value()
            z_points=self.z_points.value()
            f.box([x_5,y_5,z_5],[x_6,y_6,z_6],x_points,y_points,z_points)
            f.optimizeTrajectory()
            f.map()
            n.moveAbsolute([0,0,0])
        except:
            print("error")
            return
        
    def UP(self):
        z=self.lineEdit.text()
        z=float(z)
        z=np.abs(z)*-1
        n.moveRelative([0,0,z])
        test=n.position
        while test!=[0,0,0]:
            test_1=n.position
            test_2=n.position
            test=np.subtract(test_1,test_2)
            test=np.ndarray.tolist(test)
        self.POSITION()
    def DOWN(self):
        z=self.lineEdit.text()
        z=float(z)
        z=np.abs(z)
        n.moveRelative([0,0,z])
        test=n.position
        while test!=[0,0,0]:
            test_1=n.position
            test_2=n.position
            test=np.subtract(test_1,test_2)
            test=np.ndarray.tolist(test)
        self.POSITION()
    def LEFT(self):
        x=self.lineEdit.text()
        x=float(x)
        x=np.abs(x)
        n.moveRelative([x,0,0])
        test=n.position
        while test!=[0,0,0]:
            test_1=n.position
            test_2=n.position
            test=np.subtract(test_1,test_2)
            test=np.ndarray.tolist(test)
        self.POSITION()
    def RIGHT(self):
        x=self.lineEdit.text()
        x=float(x)*-1
        x=np.abs(x)*-1
        n.moveRelative([x,0,0])  
        test=n.position
        while test!=[0,0,0]:
            test_1=n.position
            test_2=n.position
            test=np.subtract(test_1,test_2)
            test=np.ndarray.tolist(test)
        self.POSITION()
    def AWAY(self):
        y=self.lineEdit.text()
        y=float(y)
        y=np.abs(y)*-1
        n.moveRelative([0,y,0])
        test=n.position
        while test!=[0,0,0]:
            test_1=n.position
            test_2=n.position
            test=np.subtract(test_1,test_2)
            test=np.ndarray.tolist(test)
        self.POSITION()
    def TOWARD(self):
        y=self.lineEdit.text()
        y=float(y)
        y=np.abs(y)
        n.moveRelative([0,y,0])
        test=n.position
        while test!=[0,0,0]:
            test_1=n.position
            test_2=n.position
            test=np.subtract(test_1,test_2)
            test=np.ndarray.tolist(test)
        self.POSITION()
    def POSITION(self):
        location=n.position
        location=str(location)
        self.lb1.setText(location)
    def SETZERO(self):
        n.setZero()
        location=n.position
        location=str(location)
        self.lb1.setText(location)
if __name__=="__main__":   
    app = QtWidgets.QApplication(sys.argv)
    window = UI()
    window.show()
    sys.exit(app.exec_())