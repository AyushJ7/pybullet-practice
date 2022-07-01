import numpy as np
import pybullet as p
import time
# import cv2
import math
import pybullet_data
import pidcontrol as pid
import controlpy

M = 2;   #(M) mass of the cart
m = 3;   #(m) mass of the pendulum
b = 0.5; #(b) coefficient of friction for cart
I = 0.10500391309238813; #(I) mass moment of inertia of the pendulum
g = 9.8;
l = 0.1; #(l) length to pendulum center of mass

############### Creating LQR controller to balance the robot when key is released ##############
## Using x, xDot, theta, thetaDot
A=np.array([[0, 1,  0  , 0 ],       ## [[0 ,   1  ,        0       , 0],
            [0, -0.5,-14.7, 0 ],    ## [0 , -1/M ,     -m*g/M     , 0],
            [0, 0,  0  , 1 ],       ## [0 ,   0  ,        0       , 1],
            [0,-5, -245 , 0 ]])     ## [0 ,-1/M*l, -(m+M)*g/(M*l) , 0]]
B=np.array([[0],
            [0.5],    ##[1/M]
            [0],
            [5]])     ##[1/M*l]

class SelfBalanceLQR:

    def __init__(self):
        self.xvelMin=-0.1 #x_vel
        self.xvelMax = 0
        self.Q=np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,10,0],
                        [0,0, 0, 10]])
        self.R = np.array([[0.1]])
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)
    
    def callbackLQR(self,data):

        x = data[0]
        x_dot = data[1]
        theta = data[2]
        theta_dot = data[3]

        np_x = np.array([[x],[x_dot],[theta],[theta_dot]])
        
        u_t= -(np.matmul(self.K,np_x))
        xvel = (np.matmul(A,np_x)+np.matmul(B,u_t))[1]

        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel

        return xvel-np_x[1]

def synthesizeData(p,robot):
    data = [p.getBasePositionAndOrientation(robot)[0][0],
            p.getBaseVelocity(robot)[0][0],
            p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1])[1],
            p.getBaseVelocity(robot)[1][1]]
    return data

############### Creating PID controller to move the robot when key is pressed ##############
class SelfBalancePID:
   
    def __init__(self):
        self.xvelMin=-0.01
        self.xvelMax= 0.01
        self.yMin = -0.01
        self.yMax = -0.001
        self.yPrev =0
        self.delY = 0.01
        self.Kp = 75
        self.Ki = 0
        self.Kd = 5
        self.controller=pid.PID_Controller(self.Kp,self.Ki,self.Kd)
        
    def callbackPID(self,setPoint):
        y = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1])[1]

        self.delY = y-self.yPrev # error calculation
        xvel = -self.controller.getCorrection(setPoint,y)
        # print(xvel)
        #storing variables for evaluation
        if self.delY>self.yMax:
            self.yMax = self.delY
        elif self.delY<self.yMin:
            self.yMin = self.delY

        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel
        
        if xvel >10:
            xvel =10
        elif xvel<-10:
            xvel =-10
        
        self.yPrev = y
        
        return xvel
############################   EXECUTION   ###############################

if __name__ == "__main__":
    
    id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    
    robot= p.loadURDF("/home/shivay/Projects/Self_Bal/Week3/urdf/self_balance.urdf" , [0,0,0.2])
    box = p.loadURDF("cube.urdf",[2,0,0.5],[0,0,1.73,0.7],useFixedBase=False,globalScaling=0.5)
    left_joint=0
    right_joint=1
    balanceLQR = SelfBalanceLQR()
    balancePID=SelfBalancePID()
    maxForce = 0
    mode = p.VELOCITY_CONTROL
    p.setJointMotorControl2(robot, left_joint,controlMode=mode, force=maxForce)
    p.setJointMotorControl2(robot, right_joint,controlMode=mode, force=maxForce)
    
    while(1):
        data = synthesizeData(p,robot)
        trq1 = balanceLQR.callbackLQR(data)

        keys = p.getKeyboardEvents()
        
        for k, v in keys.items():
            ############ Press 'c' to get front view of the bot ################
            pos = list(p.getBasePositionAndOrientation(robot)[0])
            orn = list(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1]))

            front_cam = [0.345*(math.cos(orn[2])), 0.345*(math.sin(orn[2])), 0.2]
            camera_pos = [pos[i]+front_cam[i] for i in range(3)]
            
            x = math.cos(orn[2]) + pos[0]
            y = math.sin(orn[2]) + pos[1]

            camera_target = [x,y,0.2]

            if k==99 and (v & p.KEY_IS_DOWN) :

                width = 512
                height = 512
                fov = 120
                aspect = width / height
                near = 0.01
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

            # if (k == 99 and (v & p.KEY_WAS_RELEASED)):
            #     cv2.imshow('rgb',rgb_opengl)
            #     cv2.waitKey(0)
            #     cv2.destroyAllWindows()

            ############### Press up key to move the bot forward #################
            if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
                setPoint = 0.1
                trq=balancePID.callbackPID(setPoint)
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [-abs(trq),abs(trq)])
                p.stepSimulation()
                
            if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
                # setPoint = 0
                p.setJointMotorControlArray(robot,[0,1] ,p.TORQUE_CONTROL, forces = [-trq1,trq1])
                p.stepSimulation()
            
            ############### Press down key to move the bot backward #################
            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
                setPoint = -0.1
                trq=balancePID.callbackPID(setPoint)
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL,forces = [abs(trq),-abs(trq)])
                p.stepSimulation()

            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
                # setPoint = 0
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [trq1,-trq1])
                p.stepSimulation()
            
            ################ Press left navigation key to turn left ##################
            if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
                t = 0.05
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [t,t])
                p.stepSimulation()

            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [trq1,-trq1])  
                p.stepSimulation()

            ################ Press right navigation key to turn left ##################
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
                t=-0.05
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [t,t])
                p.stepSimulation()

            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [trq1,-trq1])  
                p.stepSimulation()

            ############### Press 'm' to flip the bot ###################
            ############### :-\ Slightly Unstable ;-)   ###################
            if (k == 109 and (v & p.KEY_IS_DOWN)):
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [300,-300])  
                p.stepSimulation()
            if (k == 109 and (v & p.KEY_WAS_RELEASED)):
                p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [-600,600])  
                p.stepSimulation()
        
        p.setJointMotorControlArray(robot,[0,1] , p.TORQUE_CONTROL, forces = [-trq1,trq1])
 
        p.stepSimulation()
        time.sleep(1./240.)