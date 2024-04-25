#! /usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import math
import numpy as np

SIM_FREQUENCY = 240 # Hz

def configureEnvironment(gravityDown=True):
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # hide panes
    cameraSpecs = p.getDebugVisualizerCamera()
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=cameraSpecs[-4], cameraPitch=cameraSpecs[-3],
            cameraTargetPosition=cameraSpecs[-1]) # zoom in
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')
    p.setGravity(0, 0, -9.81)
    if not gravityDown:
        p.setGravity(0, 0, 9.81)

class Robot:
    def __init__(self):
        self.urdf = 'Manipulator/urdf/ManipulatorMod.urdf'

    def loadRobot(self):
        self.id = p.loadURDF(self.urdf, useFixedBase=True)
        self.n = p.getNumJoints(self.id)
        
        # Disable default velocity controllers
        p.setJointMotorControlArray(self.id, list(range(self.n)), p.VELOCITY_CONTROL, forces=[0.] * self.n)
    
    def setInitialConfiguration(self):
        angle = 10 * math.pi / 180
        for i in range(self.n):
            p.resetJointState(self.id, i, targetValue=angle)

    def flop(self, gravityDown=True):
        if gravityDown:
            p.connect(p.GUI, options='--mp4=downflop_bullet.mp4')
        else:
            p.connect(p.GUI, options='--mp4=upflop_bullet.mp4')
        configureEnvironment(gravityDown)
        self.loadRobot()
        self.setInitialConfiguration()

        time.sleep(5) # initial delay
        for i in range(1000):
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)
        
        p.disconnect()

    def getJointStates(self):
        '''
        Returns joint positions and velocities
        '''
        result = p.getJointStates(self.id, list(range(self.n)))
        q = np.array([result[i][0] for i in range(len(result))]) # position
        qd = np.array([result[i][1] for i in range(len(result))]) # velocity
        return q, qd

    def pidPositionRegulator(self, targetJoint=None, targetCartesian=None):
        '''
        '''
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        self.setInitialConfiguration()

        P = np.diag([10, 10, 5, 5, 1, 1, 1])
        D = 2 * np.sqrt(P)
        
        if targetJoint is None and targetCartesian is not None:
            targetJoint = p.calculateInverseKinematics(self.id, self.n - 1, targetCartesian)
        
        errorPrev = 0
        achieved = False
        while not achieved:
            qC, qdC = self.getJointStates()
            error = targetJoint - qC
            torque = P @ error + D @ (error - errorPrev) / SIM_FREQUENCY - 0.01 * qdC
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            errorPrev = error
            print(error)
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

        p.disconnect()

    def computedTorquePositionRegulator(self, targetJoint=None, targetCartesian=None):
        '''
        '''
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        self.setInitialConfiguration()
        
        Kp = 100 * np.eye(self.n)
        Kv = 2 * np.sqrt(Kp) # critical damping

        if targetJoint is None and targetCartesian is not None:
            targetJoint = p.calculateInverseKinematics(self.id, self.n - 1, targetCartesian)

        errorPrev = 0
        achieved = False
        while not achieved:
            qC, qdC = self.getJointStates()
            error = targetJoint - qC
            M = np.array(p.calculateMassMatrix(self.id, list(qC)))
            N = np.array(p.calculateInverseDynamics(self.id, list(qC), list(qdC), [0.] * self.n)) # sum of Coriolis and
            # gravity terms
            torque = M @ (Kv @ -qdC + Kp @ error) + N
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            errorPrev = error
            print(error)
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

    def computedTorqueTrajectoryFollower(self, trajectory, Cartesian=False):
        pass

def flopDown():
    robot = Robot()
    robot.flop()

def flopUp():
    robot = Robot()
    robot.flop(False)

def pidPositionRegulator():
    robot = Robot()
    angle = 20 * math.pi / 180
    robot.pidPositionRegulator(targetJoint=np.ones(7) * angle)

def computedTorquePositionRegulator():
    robot = Robot()
    angle = 20 * math.pi / 180
    #robot.computedTorquePositionRegulator(targetJoint=np.ones(7) * angle)
    robot.computedTorquePositionRegulator(targetCartesian=(1.0, 1.0, 1.0))

if __name__ == '__main__':
    #flopDown()
    #flopUp()
    #pidPositionRegulator()
    computedTorquePositionRegulator()
