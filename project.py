#! /usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import math
import numpy as np

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

        time.sleep(5)
        for i in range(1000):
            p.stepSimulation()
            time.sleep(1 / 240)
        
        p.disconnect()

    def getJointStates(self):
        result = p.getJointStates(self.id, list(range(self.n)))
        q = np.array([result[i][0] for i in range(len(result))]) # position
        qd = np.array([result[i][1] for i in range(len(result))]) # velocity
        return q, qd

    def pidPositionRegulator(self, qG):
        '''
        q: setpoint in joint space
        '''
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        self.setInitialConfiguration()

        P = [5, 5, 5, 5, 1, 1, 1]
        D = 1 * np.ones(self.n)
        
        errorPrev = 0
        achieved = False
        while not achieved:
            qC, qdC = self.getJointStates()
            error = qG - qC
            torque = P * error + D * (error - errorPrev) / 240 - 0.001 * qdC
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            errorPrev = error
            print(error)
            p.stepSimulation()
            time.sleep(1 / 240)

        p.disconnect()

    def computedTorquePositionRegulator(self, qG):
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        self.setInitialConfiguration()
        
        Kp = 10 * np.eye(self.n)
        Kd = 1 * np.eye(self.n)

        errorPrev = 0
        achieved = False
        while not achieved:
            qC, qdC = self.getJointStates()
            error = qG - qC
            M = np.array(p.calculateMassMatrix(self.id, list(qC)))
            N = np.array(p.calculateInverseDynamics(self.id, list(qC), list(qdC), [0.] * self.n)) # sum of Coriolis and
            # gravity terms
            torque = M @ (Kd @ error + Kp @ (error - errorPrev)) + N
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            errorPrev = error
            print(error)
            p.stepSimulation()
            time.sleep(1 / 240)

def flopDown():
    robot = Robot()
    robot.flop()

def flopUp():
    robot = Robot()
    robot.flop(False)

def pidPositionRegulator():
    robot = Robot()
    angle = 20 * math.pi / 180
    robot.pidPositionRegulator(np.ones(7) * angle)

def computedTorquePositionRegulator():
    robot = Robot()
    angle = 20 * math.pi / 180
    robot.computedTorquePositionRegulator(np.ones(7) * angle)

if __name__ == '__main__':
    #flopDown()
    #flopUp()
    #pidPositionRegulator()
    computedTorquePositionRegulator()
