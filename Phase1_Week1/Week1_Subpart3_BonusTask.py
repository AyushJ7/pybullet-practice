import pybullet as p
import pybullet_data
import time
import os
import cv2

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

file_path = os.getcwd()
file_name = "/Week1/urdf/sphere.urdf"

planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)

dropStartPos = [0,0,5]
dropStartOrientation = p.getQuaternionFromEuler([0,0,0])


c,d=0,1
e=1
x=[]
print("Enjoy Fibonacci Rainfall !!! :-)")

while True:
    #loading remaining drops other than the resetted ones
    #necessary to reach next term of the sequence
    
    for i in range(e-d,d):
        dropStartPos = [0.5*i,0,5]
        drop = p.loadURDF((file_path+file_name),dropStartPos, dropStartOrientation)
        x.append([drop,dropStartPos])

    #subtracting 1 because the function is counting plane also
    print("Number of drops present=",p.getNumBodies()-1)
    for _ in range(1000):
        p.setTimeStep(1./240.) 
        p.stepSimulation()
        time.sleep(1./240.)

    #resetting the position of previously loaded drops
    for i in x:
        p.resetBasePositionAndOrientation(i[0],i[1],dropStartOrientation)

    n1 = c+d
    n2 = d+e 
    c = d
    d = n1
    e = n2

key = cv2.waitKey(0)
if key == 27:
    p.disconnect()