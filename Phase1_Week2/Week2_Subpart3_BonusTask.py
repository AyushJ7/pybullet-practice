import pybullet as p
import pybullet_data
import time
import os

file_path = os.getcwd() 
file_name = "/Week1/urdf/sphere.urdf"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
for i in range(0,7):
	x=1+i*0.16
	bob1 = p.loadURDF(file_path+file_name,[x,0.25,0.6], globalScaling=1.5)

	base1 = p.loadURDF("cube.urdf", [x,0,2], useFixedBase=True, globalScaling=0.16)
	base2 = p.loadURDF("cube.urdf", [x,0.5,2], useFixedBase=True, globalScaling=0.16)

	p.createConstraint(base1, -1, bob1, -1, p.JOINT_POINT2POINT,[0,1,0],[0,0,0],[0,-0.25,1.4])
	p.createConstraint(base2, -1, bob1, -1, p.JOINT_POINT2POINT,[0,1,0],[0,0,0],[0,0.25,1.4])

	p.changeDynamics(bob1,-1,1,restitution=1.0)#,lateralFriction=0.5,linearDamping=0.0,angularDamping=0.0)
					
p.applyExternalForce(bob1,-1,[250,0,0],[x,0.25,0.6],p.WORLD_FRAME)

while True:
	p.stepSimulation()
	time.sleep(1./240.)