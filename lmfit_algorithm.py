from lmfit import Parameters
from lmfit import Model
import numpy as np
import pandas as pd

df = pd.read_csv(r'Data\Resolution\R=8 N=200')

scale = True

x = df['X']
y = df['Y']
z = df['Z']
bx = df['Bx']
by = df['By']
bz = df['Bz']

nPoints = len(x)

if scale:
    # convert to Tesla, meters
    bx /= 1e9
    by /= 1e9
    bz /= 1e9
    x /= 1e3
    y /= 1e3
    z /= 1e3

x = x.to_numpy()
y = y.to_numpy()
z = z.to_numpy()
bx = bx.to_numpy()
by = by.to_numpy()
bz = bz.to_numpy()

# reshape data side by side by column
xData = np.column_stack((x,y,z,bx))
yData = np.column_stack((x,y,z,by))
zData = np.column_stack((x,y,z,bz))

# add the value for d for each component in a new column
xAddedD = np.full((nPoints, 5), 1, dtype=float)
xAddedD[:,:-1] = xData

yAddedD = np.full((nPoints, 5), 2, dtype=float)
yAddedD[:,:-1] = yData

zAddedD = np.full((nPoints, 5), 3, dtype=float)
zAddedD[:,:-1] = zData

# stack the componenets into a column array, waiting to be separated by x,y,z,d. Now 3N greater length
dataSeparated = np.vstack((xAddedD,yAddedD,zAddedD))

xFinal = dataSeparated[:,0]
yFinal = dataSeparated[:,1]
zFinal = dataSeparated[:,2]
magFieldDataFinal = dataSeparated[:,3]
dFinal = dataSeparated[:,4]


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


def main():
    dipole_model = Model(magField, independent_vars=['x','y','z','d'])

    params = Parameters()
    params.add('a',value=0.01, min=-1.0, max=1.0)
    params.add('b',value=0.01, min=-1.0, max=1.0)
    params.add('c',value=0.01, min=-1.0, max=1.0)
    params.add('mx',value=0.5, min=-10.0, max=10.0)
    params.add('my',value=0.5, min=-10.0, max=10.0)
    params.add('mz',value=0.5, min=-10.0, max=10.0)

    result = dipole_model.fit(magFieldDataFinal, params, x=xFinal,y=yFinal,z=zFinal,d=dFinal)
    print(result.fit_report())



if __name__ == '__main__':
    main()
