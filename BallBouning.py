import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# sim engine parameters
p.setPhysicsEngineParameter(fixedTimeStep=0.0005,
                            erp=0.0,
                            contactERP=0.0,
                            frictionERP=0.0,
                            restitutionVelocityThreshold=0.0)

# True/False
isPlaneFromURDF = False

if isPlaneFromURDF:
    # plane from urdf
    planeId = p.loadURDF("res/plane.urdf",
                         useMaximalCoordinates=0,       # maxco disabled. since it's not supported in C++ API
                         flags=p.URDF_USE_INERTIA_FROM_FILE)
else:
    # plane from createmultibody
    planeShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                          halfExtents=[20, 20, 0.05])

    planeId = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=planeShapeId,
                                basePosition=[0., 0., -0.05],
                                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                                useMaximalCoordinates=0,  # maxco disabled. since it's not supported in C++ API
                                )

p.changeDynamics(bodyUniqueId=planeId,
                 linkIndex=-1,
                 lateralFriction=0.0,
                 rollingFriction=0.0,
                 spinningFriction=0.0,
                 linearDamping=0,
                 angularDamping=0,
                 mass=0,
                 localInertiaDiagonal=[0, 0, 0],
                 restitution=1.0)

# ball from URDF
ballId = p.loadURDF("res/ball.urdf",
                    basePosition=[-1., 0., 5.],
                    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                    useMaximalCoordinates=0,  # maxco disabled. since it's not supported in C++ API
                    flags=p.URDF_USE_INERTIA_FROM_FILE)

p.changeDynamics(bodyUniqueId=ballId,
                 linkIndex=-1,
                 lateralFriction=0.0,
                 rollingFriction=0.0,
                 spinningFriction=0.0,
                 linearDamping=0,
                 angularDamping=0,
                 restitution=1.0)

# ball from create multibody
ballShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE,
                                     radius=0.1)

ballId2 = p.createMultiBody(baseMass=10,
                            baseCollisionShapeIndex=ballShapeId,
                            basePosition=[1., 0., 5.],
                            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                            useMaximalCoordinates=0,  # maxco disabled. since it's not supported in C++ API
                            )

p.changeDynamics(bodyUniqueId=ballId2,
                 linkIndex=-1,
                 lateralFriction=0.0,
                 rollingFriction=0.0,
                 spinningFriction=0.0,
                 linearDamping=0,
                 angularDamping=0,
                 restitution=1.0)

# simulation step
for t in range (10000):
    # simulation step
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
