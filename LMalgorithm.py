from scipy.optimize import root
import numpy as np
import pandas as pd
import time

# read in data
df = pd.read_csv(r"Data\Resolution\R=6 N=150")

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

# options={'xtol': 1.49012e-20, 'ftol': 1.49012e-20}

def solve_LM():
    x0 = np.array([0.05,0.05,0.05,1.2,1.2,1.2])
    sol = root(vector_func, x0, method='lm', options={'xtol': 1.49012e-20, 'ftol': 1.49012e-20})
    print_solution(sol)

def main():

    startAlgo = time.time()
    solve_LM()
    endAlgo = time.time() - startAlgo

    print(f"\nAlgorithm Time: {endAlgo} s")

if __name__ == "__main__":
    main()