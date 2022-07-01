import pybullet as p
import pybullet_data

p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

carpos = [0, 0, 0.1]
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])

numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))

targetVel = 3  #rad/s
maxForce = 100 #Newton
p.setRealTimeSimulation(1)

while (1):
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == 97 and (v & p.KEY_WAS_RELEASED)):
            targetVel=targetVel +1
            print("New Target Velocity = ",targetVel)

        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = targetVel,
                                        force = maxForce)
            p.stepSimulation()
            
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = -targetVel,
                                        force = maxForce)
            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
        
        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            Vel=3
            for joint in range(2,5,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel/3,
                                        force = maxForce)
            for joint in range(3,6,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()

        if(k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            Vel=3
            for joint in range(2,5,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            for joint in range(3,6,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel/3,
                                        force = maxForce)
            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
        
        if (k==114 and (v & p.KEY_IS_DOWN)):
            Vel = 15
            for joint in range(2,5,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            for joint in range(3,6,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = -Vel,
                                        force = maxForce)
            p.stepSimulation()

        if (k == 114 and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()

p.getContactPoints(car)
p.disconnect()