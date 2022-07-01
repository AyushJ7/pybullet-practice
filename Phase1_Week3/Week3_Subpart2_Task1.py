import pybullet as p
import pybullet_data
import os
import time
import math

file_path = os.getcwd() 
file_name = "/Week3/urdf/R_planar_robot.urdf"
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_path+file_name,useFixedBase=True)

p.resetBasePositionAndOrientation(robot, [0, 0, 1], [0, 0, 0, 0.707])

p.setGravity(0,0,-10)

point_A = [0,2,1.1]	
point_B = [0,-2,1.1]
p.addUserDebugLine(point_A,point_B,[1,0,0],2)

# point_A = [0,2,1.5]	
# point_B = [0,-1,3.232]
# p.addUserDebugLine(point_A,point_B,[1,0,0],2)

l1 = 1 
l2 = 1 

def Forward_kinematics(angle_1,angle_2):

	y= l1*(math.cos(angle_1)) + l2*(math.cos(angle_1+angle_2))
	z= l1*(math.sin(angle_1)) + l2*(math.sin(angle_1+angle_2))
	return [0,y,z]

def Inverse_kinematics(target):
	y = target[1]
	z = target[2]
	a = y**2 + z**2
	b = l1**2 + l2**2

	########## Method - 1 ############
	if a==0:
		print("Infinite possibilities!!!!")
	else:
		angle_1=math.asin((a+l1**2 - l2**2)/(2*l1*math.sqrt(a))) - (math.atan2(y,z))

	angle_2=math.acos((a-b)/(2*l1*l2))
	return angle_1,angle_2

	########## Method - 2 ############
	'''
	angle_2= (math.acos((a-b)/(2*l1*l2)))

	tan_alpha = (l2*math.sin(angle_2))/(l1+(l2*math.cos(angle_2)))
	c = (z/y)
	if z==0 and y ==0:
		print("Infinite possibilities!!!!")
	else:
		angle_1 = math.atan2((c-tan_alpha),(1+c*tan_alpha))
	return angle_1,angle_2
	'''
while(True):

	### Following a line of the form z = my + c with negative slope 
	'''
	for i in range(-625,1950):
		z = i/1000
		y = 1.54 - 0.57*z

		angle_1,angle_2  = Inverse_kinematics([0,y,z])
		# print(math.degrees(angle_1),math.degrees(angle_2))
		p.setJointMotorControl2(bodyIndex=robot,
								jointIndex=0,
								controlMode =p.POSITION_CONTROL,
								targetPosition= -angle_1,
								force=500)

		p.setJointMotorControl2(bodyIndex=robot,
								jointIndex=1,
								controlMode =p.POSITION_CONTROL,
								targetPosition= -angle_2,
								force=500)
		
		p.stepSimulation()
		time.sleep(1./240.)
	'''
	### Following a line parallel to y-axis 
	
	for i in range(1997,-1997,-1):
		z = i/1000
		y = 0.05
		
		angle_1,angle_2  = Inverse_kinematics([0,y,z])
		# print(math.degrees(angle_1),math.degrees(angle_2))
		p.setJointMotorControl2(bodyIndex=robot,
								jointIndex=0,
								controlMode =p.POSITION_CONTROL,
								targetPosition= -angle_1,
								force=500)

		p.setJointMotorControl2(bodyIndex=robot,
								jointIndex=1,
								controlMode =p.POSITION_CONTROL,
								targetPosition= -angle_2,
								force=500)
		
		p.stepSimulation()
		time.sleep(1./240.)

### Maximum length of the line segment that the robot can trace will be approximately 4 
### and the line segment will be parallel to y-axis passing through the base of the robot.
### Basically it is extreme case of line of the form z = my + c with m = 0    