'''
1. Press "c" to capture the front image
2. Press page up to move forward 
3. Press page down to move backward
4. Press left arrow to move left
5. Press right arrow to move right
'''
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import os
import math

p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

file_path = os.getcwd() 
file_name = "/Week1/urdf/sphere.urdf"

carpos = [0, 0, 0.1]
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])

numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))

targetVel = 3  #rad/s
maxForce = 100 #Newton
p.setRealTimeSimulation(1)

point=p.loadURDF(file_path+file_name,[-0.5,0,0.7],useFixedBase=False,globalScaling=5)
box = p.loadURDF("cube.urdf",[0,5,1],[0,0,1.73,0.7],useFixedBase=False,globalScaling=2)

while (1):
    keys = p.getKeyboardEvents()
    
    for k, v in keys.items():
        
        pos = list(p.getBasePositionAndOrientation(car)[0])
        orn = list(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(car)[1]))

        front_cam = [0.345*(math.cos(orn[2])), 0.345*(math.sin(orn[2])), 0.4]
        camera_pos = [pos[i]+front_cam[i] for i in range(3)]
        
        x = math.cos(orn[2]) + pos[0]
        y = math.sin(orn[2]) + pos[1]

        camera_target = [x,y,0.4]

        if k==99 and (v & p.KEY_IS_DOWN) :

            width = 512
            height = 512
 
            fov = 120
            aspect = width / height
            near = 0.02
            far = 15

            view_matrix = p.computeViewMatrix(camera_pos,camera_target, [0, 0, 1])
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
            
            # Get depth values using the OpenGL renderer
            images = p.getCameraImage(width,
                                    height,
                                    view_matrix,
                                    projection_matrix,
                                    shadow=True,
                                    renderer=p.ER_BULLET_HARDWARE_OPENGL)

            
            rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            # bgr_opengl = np.flip(rgb_opengl,2)

            
        if (k == 99 and (v & p.KEY_WAS_RELEASED)):
            cv2.imshow('rgb',rgb_opengl)
            # cv2.imshow('rgb',bgr_opengl)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        if (k == 97 and (v & p.KEY_WAS_RELEASED)):
            targetVel=targetVel + 1
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