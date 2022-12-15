import pandas as pd
import numpy as np
from mayavi import mlab

df = pd.read_csv(r"backgroundTest3")

x = df['X']
y = df['Y']
z = df['Z']
bx = df['Bx']
by = df['By']
bz = df['Bz']

bx /= 1e9
by /= 1e9
bz /= 1e9

# convert to numpy arrays
x = x.to_numpy()
y = y.to_numpy()
z = z.to_numpy()
bx = bx.to_numpy()
by = by.to_numpy()
bz = bz.to_numpy()

# vector plot. From mayaVI
mlab.figure('Vector Field')
mlab.quiver3d(x,y,z,bx,by,bz, colormap='gist_rainbow')
mlab.axes(extent=[-300,300,-300,300,-300,300])
mlab.outline()
mlab.show()


# compute averages
averageBx = np.average(bx) 
averageBy = np.average(by)
averageBz = np.average(bz)

print(f'Average Bx: {averageBx} (T)')
print(f'Average By: {averageBy} (T)')
print(f'Average Bz: {averageBz} (T)')









