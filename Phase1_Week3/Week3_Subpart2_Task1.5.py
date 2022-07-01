import pybullet as p
import pybullet_data
import os
import time

file_path = os.getcwd() 
file_name = "/Week3/urdf/R_planar_robot.urdf"
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_path+file_name,useFixedBase=True)

p.resetBasePositionAndOrientation(robot, [0, 0, 1], [0, 0, 0, 0.707])
p.setGravity(0,0,-10)

point_A = [0,2,1.15]	
point_B = [0,-2,1.15]
p.addUserDebugLine(point_A,point_B,[1,0,0],2)

while(True):
	### Following a line parallel to y-axis
	for i in range(1950,-1951,-1):
		z = i/1000
		y = 1.15
		jointPoses = p.calculateInverseKinematics(robot,2,[0,z,y])
		for i in range(0,2):
			p.setJointMotorControl2(bodyIndex=robot, 
									jointIndex=i,
									controlMode=p.POSITION_CONTROL,
									targetPosition=jointPoses[i],
									force=500)
		p.stepSimulation()
		time.sleep(1./240.)
	