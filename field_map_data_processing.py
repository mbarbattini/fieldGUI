import numpy as np
from lmfit import Parameters
from lmfit import Model
from scipy.optimize import root

class FieldMapDataProcessing:

    def __init__(self):
        self.B = np.empty((0,3))
        self.coords = np.empty((0,3))

    def importData(self):
        pass

    def lm(self):
        """
        Uses the Levenberg-Marquardt algorithm to calculate the effective dipole moment.
        """
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

        print(f"x[0] = a  = {sol.x[0]} m")
        print(f"x[1] = b  = {sol.x[1]} m")
        print(f"x[2] = c  = {sol.x[2]} m")
        print(f"x[3] = mx = {sol.x[3]}")
        print(f"x[4] = my = {sol.x[4]}")
        print(f"x[5] = mz = {sol.x[5]}")

        mag = np.sqrt(sol.x[3]**2+sol.x[4]**2+sol.x[5]**2)

        print(f"\nmagnitude: {mag} Am^2")


    def lmfit(self):
        """
        Uses the lmfit algorithm to calulate the effective dipole moment.
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
        print(result.fit_report())


    def plotField(self):
        pass

    def exportData(self):
        pass
