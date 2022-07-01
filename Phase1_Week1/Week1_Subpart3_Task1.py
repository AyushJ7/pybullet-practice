import pybullet as p
import os
import time
import pybullet_data
import cv2
import math

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
file_path = os.getcwd() 
file_name_1 = "/Week1/urdf/r2d2.urdf"
file_name_2 = "/Week1/urdf/dabba.urdf"

planeId = p.loadURDF("plane.urdf")
roboStartPos = [0,0,1]
roboStartOrientation = p.getQuaternionFromEuler([0,0,0])
sampleId = p.loadURDF((file_path+file_name_1),roboStartPos, roboStartOrientation)

dabbaStartPos=[2,2,1]
dabbaStartOrientation = p.getQuaternionFromEuler([0,0,0])
dabbaID = p.loadURDF((file_path+file_name_2),dabbaStartPos, dabbaStartOrientation)
d=0
while True:
    d=0
    for t in range(11):
        d+=1
        p.setGravity(t*0.98/math.sqrt(2),t*0.98/math.sqrt(2),0)
        #Using the obtained value of g for 50 timesteps in the loop below
        for i in range(50):
            p.stepSimulation()
            time.sleep(1.0/240.0)
            #Value of g gets updated after this loop
        print(d,"-->","g=",math.sqrt((t*0.98/math.sqrt(2))**2 + (t*0.98/math.sqrt(2))**2))
    
    # Resetting the position of both bots
    p.resetBasePositionAndOrientation(sampleId, roboStartPos, roboStartOrientation)
    p.resetBasePositionAndOrientation(dabbaID, dabbaStartPos, dabbaStartOrientation)

key = cv2.waitKey(0)
if key == 27:
    p.disconnect()