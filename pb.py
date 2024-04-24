#! /usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import math
import numpy as np

def configureEnvironment():
    physicsClient = p.connect(p.GUI, options=
            '--mp4=flop.mp4')
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF('plane.urdf')

class Robot:
    def __init__(self):
        configureEnvironment()
        self.id = p.loadURDF('Manipulator/urdf/ManipulatorMod.urdf',
                useFixedBase=True)
        self.n = p.getNumJoints(self.id)

        # Disable default velocity controllers
        p.setJointMotorControlArray(self.id, list(range(self.n)), p.VELOCITY_CONTROL,
                forces=[0.] * self.n)

    def setInitialConfiguration(self):
        angle = 10 * math.pi / 180
        for i in range(self.n):
            p.resetJointState(self.id, i, targetValue=angle)

    def flop(self, gravityDown=True):
        if not gravityDown:
            p.setGravity(0, 0, 9.81)
        self.setInitialConfiguration()
        time.sleep(5)
        for i in range(1000):
            p.stepSimulation()
            time.sleep(1 / 240)
        if not gravityDown:
            p.setGravity(0, 0, -9.81)

def flop():
    robot = Robot()
    robot.flop()
    robot.flop(False)

def demo():
    # Configure environment
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF('plane.urdf')

    # Load manipulator
    position = [0, 0, 0]
    orientation = p.getQuaternionFromEuler([0, 0, 0])
    #manipulatorId = p.loadURDF('Manipulator/urdf/Manipulator.urdf', position,
    #        orientation, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
    #manipulatorId = p.loadURDF('Manipulator/urdf/Manipulator.urdf', position,
    #        orientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
    manipulatorId = p.loadURDF('Manipulator/urdf/ManipulatorMod.urdf')
    n = p.getNumJoints(manipulatorId)
    angle = 10 * math.pi / 180
    for i in range(1, n):
        p.resetJointState(manipulatorId, i, targetValue=angle)
    #for i in range(n):
    #    print(p.getJointInfo(manipulatorId, i))

    # Control
    # Disable default velocity controller
    #p.setJointMotorControlArray(manipulatorId, list(range(n)), p.VELOCITY_CONTROL,
    #        forces=[0., 0., 0., 0., 0., 0., 0.])
    for i in range(1, n):
        p.setJointMotorControl2(manipulatorId, i, p.VELOCITY_CONTROL, force=0)
    # Zero torque control
    p.setJointMotorControlArray(manipulatorId, list(range(1, n)), p.TORQUE_CONTROL,
            forces=[0., 0., 0., 0., 0., 0., 0.])

    time.sleep(5)
    for i in range(10000):
        p.setJointMotorControlArray(manipulatorId, list(range(1, n)), p.TORQUE_CONTROL,
                forces=[0., 0., 0., 0., 0., 0., 0.])
        p.stepSimulation()
        time.sleep(1 / 240)
    p.disconnect()

if __name__ == '__main__':
    #run()
    flop()
