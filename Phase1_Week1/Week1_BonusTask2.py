import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, -10)

tex = p.loadTexture("uvmap.png")
planeId = p.loadURDF("plane.urdf", [0,0,-1])
 
boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

bunnyId = p.loadSoftBody("torus_torus_textured.obj", simFileName="torus.vtk",scale=1.25,basePosition=[0,0,0],
                            baseOrientation=[1,1,0,0],
                            mass = 3, useNeoHookean = 1, NeoHookeanMu = 400, 
                            NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, 
                            collisionMargin = 0.001, useSelfCollision = 1, 
                            frictionCoeff = 0.5, repulsionStiffness = 800)
p.changeVisualShape(bunnyId, -1, rgbaColor=[1, 0.7, 0.4, 1])

bunnyId2 = p.loadSoftBody("torus_torus_textured.obj", simFileName="torus.vtk",scale=1,basePosition=[0,-0.75,0],
                            baseOrientation=[0,0.5,0.5,0],
                            mass = 3, useNeoHookean = 1, NeoHookeanMu = 400, 
                            NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, 
                            collisionMargin = 0.001, useSelfCollision = 1, 
                            frictionCoeff = 0.5, repulsionStiffness = 800)
p.changeVisualShape(bunnyId2, -1, rgbaColor=[1,0,1,1])

bunnyId3 = p.loadSoftBody("torus_torus_textured.obj", simFileName="torus.vtk",scale=1,basePosition=[0,0.75,0],
                            baseOrientation=[0,0.5,0.5,0],
                            mass = 3, useNeoHookean = 1, NeoHookeanMu = 400, 
                            NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, 
                            collisionMargin = 0.001, useSelfCollision = 1, 
                            frictionCoeff = 0.5, repulsionStiffness = 800)
p.changeVisualShape(bunnyId3, -1, rgbaColor=[0,1,1,1])

p.setPhysicsEngineParameter(sparseSdfVoxelSize=10)
p.setRealTimeSimulation(0)

while True:
  p.stepSimulation()
