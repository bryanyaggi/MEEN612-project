#! /usr/bin/env python3

import math
import numpy as np
import pybullet as p
import pybullet_data
import roboticstoolbox as rtb
from spatialmath import SE3, SO3
from spatialmath.base import r2q, rotz
import time

import unittest
from scipy.spatial.transform import Rotation

'''
Notes

Quaternions
spatialmath uses scalar-first format: (w, x, y, z)
PyBullet and SciPy use scalar-last format: (x, y, z, w)
'''

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
        '''
        Non-singular configuration
        '''
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

    def getEndPose(self):
        '''
        Returns pose of end joint
        '''
        state = p.getLinkState(self.id, self.n - 1, computeForwardKinematics=True)
        position = state[4]
        orientation = state[5]
        return position, orientation

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
            print(self.getEndPose()[0])
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

        p.disconnect()

    def computedTorquePositionRegulator(self, targetJoint=None, targetCartesian=None, useCartesianOrientation=False):
        '''
        targetJoint is list of joint angles
        targetJoint is SE3 transform
        useCartesianOrientation indicates whether Cartesian orientation is required for inverse kinematics
        '''
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        self.setInitialConfiguration()
        
        Kp = 100 * np.eye(self.n)
        Kv = 2 * np.sqrt(Kp) # critical damping

        if targetJoint is None and targetCartesian is not None:
            maxIterations = 100
            if useCartesianOrientation:
                quat = Rotation.from_matrix(targetCartesian.R).as_quat()
                targetJoint = p.calculateInverseKinematics(self.id, self.n - 1, targetCartesian.t, quat,
                        maxNumIterations=maxIterations)
            else:
                targetJoint = p.calculateInverseKinematics(self.id, self.n - 1, targetCartesian.t,
                        maxNumIterations=maxIterations)

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
            print(self.getEndPose()[0])
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

        p.disconnect()

    def computedTorqueTrajectoryFollower(self, trajectoryJoint=None, trajectoryCartesian=None,
            useCartesianOrientation=False):
        '''
        trajectoryJoint is joint space trajectory of type rtb.tools.trajectory.Trajectory
        trajectoryCartesian is a list of SE3 transforms
        useCartesianOrientation indicates whether Cartesian orientation is required for inverse kinematics
        '''
        p.connect(p.GUI)
        configureEnvironment()
        self.loadRobot()
        
        if trajectoryJoint is None and trajectoryCartesian is not None:
            cartesian = True
            steps = len(trajectoryCartesian)
            # Set end effector at start
            if useCartesianOrientation:
                quat = Rotation.from_matrix(trajectoryCartesian[0].R).as_quat()
                qPrev = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[0].t, quat)
            else:
                qPrev = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[0].t)
            #qPrev = np.array(qPrev)
            for i in range(self.n):
                p.resetJointState(self.id, i, targetValue=qPrev[i])
            qT = np.zeros(self.n)
            qdPrev = np.zeros(self.n)

        else:
            cartesian = False
            steps = len(trajectoryJoint)
            # Set end effector at start
            for i in range(self.n):
                p.resetJointState(self.id, i, targetValue=trajectoryJoint.s[0][i])
        
        Kp = 100 * np.eye(self.n)
        Kv = 2 * np.sqrt(Kp) # critical damping

        for i in range(steps):
            qC, qdC = self.getJointStates()
            if not cartesian:
                e = trajectoryJoint.s[i] - qC
                ed = trajectoryJoint.sd[i] - qdC
                qddT = trajectoryJoint.sdd[i]
            else:
                if useCartesianOrientation:
                    quat = Rotation.from_matrix(trajectoryCartesian[i].R).as_quat()
                    qT[:] = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[i].t, quat)
                else:
                    qT[:] = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[i].t)
                # Calculate derivatives using finite difference
                if i == 0:
                    # use forward difference
                    pass
                elif i == steps - 1:
                    # use backward difference
                    pass
                else:
                    # use center difference
                    pass
                # forward difference
                qdT = (qT - qPrev) / (1 / SIM_FREQUENCY)
                qddT = (qdT - qdPrev) / (1 / SIM_FREQUENCY)

                # update
                qPrev = qT
                qdPrev = qdT

                e = qT - qC
                ed = qdT - qdC

            #print(e)
            print(self.getEndPose()[0])
            M = np.array(p.calculateMassMatrix(self.id, list(qC)))
            N = np.array(p.calculateInverseDynamics(self.id, list(qC), list(qdC), [0.] * self.n)) # sum of Coriolis and
            # gravity terms
            torque = M @ (qddT + Kv @ ed + Kp @ e) + N
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
    #robot.computedTorquePositionRegulator(targetJoint=np.ones(7) * angle)
    T = SE3(0, 0, 0.2) * SE3.Rx(math.pi / 2)
    robot.computedTorquePositionRegulator(targetCartesian=T, useCartesianOrientation=True)

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
    robot = Robot()
    T0 = SE3(-0.2, -0.2, 0.2) * SE3.Rx(math.pi / 2)
    Tf = SE3(0.2, -0.2, 0.2) * SE3.Rx(math.pi / 2)
    duration = 10
    t = np.linspace(0, duration, duration * SIM_FREQUENCY)
    trajectory = rtb.tools.trajectory.ctraj(T0, Tf, t)
    #print(trajectory)
    robot.computedTorqueTrajectoryFollower(trajectoryCartesian=trajectory, useCartesianOrientation=True)
    #quat = r2q(trajectory[0].R)
    #print(T0.R)
    #quat = Rotation.from_matrix(Tf.R).as_quat()
    #print(quat)
    #robot.computedTorquePositionRegulator(targetCartesian=(trajectory[-1].t, quat))

class Test(unittest.TestCase):
    def testQuaternionConversion(self):
        angle = 30 * math.pi / 180
        #R = SO3.Rz(angle)
        R = rotz(angle)
        print(R)
        q = r2q(R)
        print(q)

if __name__ == '__main__':
    #flopDown()
    #flopUp()
    #pidPositionRegulator()
    #computedTorquePositionRegulator()
    #computedTorqueTrajectoryFollower()
    computedTorqueTrajectoryFollowerC()
