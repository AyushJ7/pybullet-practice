import pybullet as p
import pybullet_data
import time
import os

print("For velocityControl simulation, press 1")
print("For torqueControl simulation, press 2")
n = int(input("Enter: "))
while n != 1 and n != 2:
    new = int(input("Wrong choice,try again : "))
    if new==1 or new==2:
        n=new
        break

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

ramp=p.loadURDF("/Week2/wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

file_path = os.getcwd()
file_name = "/Week2/stage.urdf"
stage=p.loadURDF(file_path+file_name , [-4,0.35,0.589])

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

print("Number of joints =" , p.getNumJoints(husky))
print("Joint information given below :")

for i in range(p.getNumJoints(husky)):
    print(p.getJointInfo(husky,i))

def Torque_control():
    optimal_torque_value = -250
    for i in range(2,6):
        p.setJointMotorControl2(bodyUniqueId=husky,jointIndex=i,
                                controlMode=p.TORQUE_CONTROL,
                                force = optimal_torque_value)

def Velocity_control():
    maxForce = 100
    optimal_velocity_value = -15
    for i in range(2,6):
        p.setJointMotorControl2(bodyUniqueId=husky,jointIndex=i,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity = optimal_velocity_value,
                                force = maxForce)
c=0
while (1):
    c+=1
    time.sleep(0.01)
    if n==1:
        Velocity_control()
    if n==2:
        Torque_control()

    p.stepSimulation()
    if c==100:
        c=0
        print("Linear Velocity of Base = ",p.getBaseVelocity(husky)[0])
        print("Angular Velocity of Base = ",p.getBaseVelocity(husky)[1])

        print("Base link state information :-")
        print(p.getLinkState(husky,0))

p.disconnect()