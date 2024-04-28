#! /usr/bin/env python3

import math
import numpy as np
import pybullet as p
import pybullet_data
import roboticstoolbox as rtb
from spatialmath import SE3
import time

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

        achieved = False
        while not achieved:
            qC, qdC = self.getJointStates()
            error = targetJoint - qC
            M = np.array(p.calculateMassMatrix(self.id, list(qC)))
            N = np.array(p.calculateInverseDynamics(self.id, list(qC), list(qdC), [0.] * self.n)) # sum of Coriolis and
            # gravity terms
            torque = M @ (Kv @ -qdC + Kp @ error) + N
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            print(error)
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

        p.disconnect()

    def computedTorqueTrajectoryFollower(self, trajectoryJoint=None, trajectoryCartesian=None):
        '''
        '''
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        self.setInitialConfiguration()
        
        Kp = 100 * np.eye(self.n)
        Kv = 2 * np.sqrt(Kp) # critical damping

        if trajectoryJoint is None and trajectoryCartesian is not None:
            cartesian = True
            steps = len(trajectoryJoint)
        else:
            cartesian = False
            steps = len(trajectoryCartesian)

        for i in range(steps):
            qC, qdC = self.getJointStates()
            if not cartesian:
                e = trajectoryJoint.s[i] - qC
                ed = trajectoryJoint.sd[i] - qdC
            else:
                qT = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[i][:3, 3],
                        trajectoryCartesian[i])
                if i == 0:
                    # use forward difference
                elif i == steps - 1:
                    # use backward difference
                else:
                    # use center difference

            M = np.array(p.calculateMassMatrix(self.id, list(qC)))
            N = np.array(p.calculateInverseDynamics(self.id, list(qC), list(qdC), [0.] * self.n)) # sum of Coriolis and
            # gravity terms
            torque = M @ (trajectoryJoint.sdd[i] + Kv @ ed + Kp @ e) + N
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

        p.disconnect()

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

def computedTorqueTrajectoryFollower():
    robot = Robot()
    initialAngle = 10 * math.pi / 180
    finalAngle = 45 * math.pi / 180
    q0 = np.ones(7) * initialAngle
    qf = np.ones(7) * finalAngle
    duration = 10
    t = np.linspace(0, duration, duration * SIM_FREQUENCY)
    trajectory = rtb.tools.trajectory.jtraj(q0, qf, t)
    robot.computedTorqueTrajectoryFollower(trajectoryJoint=trajectory)

def computedTorqueTrajectoryFollowerC():
    T0 = np.eye(4)
    T0[:3, 3] = [-1, 0, 1]
    R = np.array([
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]])
    T0[:3, :3] = R
    print(T0)
    Tf = np.eye(4)
    Tf[:3, 3] = [1, 0, 1]
    Tf[:3, :3] = R
    T0 = SE3(-1, 0, 1) * SE3.Rx(math.pi)
    Tf = SE3(1, 0, 1) * SE3.Rx(math.pi)
    print(T0)
    duration = 10
    t = np.linspace(0, duration, duration * SIM_FREQUENCY)
    trajectory = rtb.tools.trajectory.ctraj(T0, Tf, t)
    robot.computedTorqueTrajectoryFollower(trajectoryCartesian=trajectory)

if __name__ == '__main__':
    #flopDown()
    #flopUp()
    #pidPositionRegulator()
    #computedTorquePositionRegulator()
    #computedTorqueTrajectoryFollower()
    computedTorqueTrajectoryFollowerC()
