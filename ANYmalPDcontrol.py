import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# sim engine parameters
p.setPhysicsEngineParameter(fixedTimeStep=0.005,
                            numSolverIterations=50,
                            erp=0.2,
                            contactERP=0.2,
                            frictionERP=0.2)

planeId = p.loadURDF("res/plane.urdf")
p.changeDynamics(bodyUniqueId=planeId,
                 linkIndex=-1,
                 lateralFriction=1.0)

# variables
numJoint = 0    # will be updated!
numrow = 15
robotIds = []

# joint configuration
jointConfig = np.array([
    0.03, 0.4, -0.8, 0,   # LF
    -0.03, 0.4, -0.8, 0,  # RF
    0.03, -0.4, 0.8, 0,   # LH
    -0.03, -0.4, 0.8, 0,  # RH
    0
])

# robot load
for i in range(0, numrow):
    for j in range(0, numrow):
        startPos = [(i- numrow * 0.5) *  2.0 , (j - numrow * 0.5)* 2.0, 0.54]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        robotId = p.loadURDF("res/anymal.urdf",
                             startPos,
                             startOrientation,
                             useMaximalCoordinates=0,
                             flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_IMPLICIT_CYLINDER | p.URDF_USE_SELF_COLLISION)
        robotIds.append(robotId)

        numJoint = p.getNumJoints(robotId)
        jointIdx = np.arange(numJoint)
        zeros = np.zeros(numJoint)

        for k in range(0, numJoint):
            p.resetJointState(bodyUniqueId=robotId,
                              jointIndex=k,
                              targetValue=jointConfig[k])
            p.changeDynamics(bodyUniqueId=planeId,
                             linkIndex=k,
                             lateralFriction=0.8)

        p.setJointMotorControlArray(bodyUniqueId=robotId,
                                    jointIndices=jointIdx,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=zeros)


# gains
kp = 40
kd = 1.0

# simulation step
for t in range (10000):
    for id in robotIds:
        # control
        jointIdx = np.arange(numJoint)

        jointStates = p.getJointStates(bodyUniqueId=id,
                                       jointIndices=jointIdx)
        jointPositions = np.array([s[0] for s in jointStates])
        jointVelocities = np.array([s[1] for s in jointStates])

        jointTorques = kp * (jointConfig - jointPositions) - kd * jointVelocities

        p.setJointMotorControlArray(bodyUniqueId=id,
                                    jointIndices=jointIdx,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=jointTorques)

    # simulation step
    p.stepSimulation()
    time.sleep(1./240.)
    info = p.getContactPoints()

cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()
