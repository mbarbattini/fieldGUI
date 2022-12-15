from auto_map import RotationalMap, FieldMap
import time

r = RotationalMap()
f = FieldMap()

def main():

    start = time.time()

    r.rotationalPlane(6,[50,50,-120],[-50,-50,-120],4,4,"Planes N=45 Stepper Motor", backgroundSubtract=True)

    # f.plane([50,50,-120],[-50,-50,-120],15,15)
    # f.optimizeTrajectory()
    # f.map()
    
    end = (time.time() - start) / 60
    print(f"Total mapping time: {end} min")

if __name__ == '__main__':
    main()
