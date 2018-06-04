import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# sim engine parameters
p.setPhysicsEngineParameter(fixedTimeStep=0.005,
                            erp=0.0,
                            contactERP=0.0,
                            frictionERP=0.0,
                            restitutionVelocityThreshold=0.0)

planeId = p.loadURDF("res/plane.urdf")
p.changeDynamics(bodyUniqueId=planeId,
                 linkIndex=-1,
                 lateralFriction=0.0,
                 rollingFriction=0.0,
                 spinningFriction=0.0,
                 restitution=1.0)

# variables
numrow = 7
robotIds = []

# robot load
for i in range(0, numrow):
    for j in range(0, numrow):
        startPos = [(i- numrow * 0.5) *  2.0 , (j - numrow * 0.5)* 2.0, 5.0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        robotId = p.loadURDF("res/ball.urdf",
                             startPos,
                             startOrientation,
                             useMaximalCoordinates=1,
                             flags=p.URDF_USE_INERTIA_FROM_FILE)

        p.changeDynamics(bodyUniqueId=planeId,
                         linkIndex=-1,
                         lateralFriction=0.0,
                         rollingFriction=0.0,
                         spinningFriction=0.0,
                         restitution=1.0)

        robotIds.append(robotId)


# simulation step
for t in range (10000):
    # simulation step
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
