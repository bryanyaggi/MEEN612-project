#! /usr/bin/env python3

import math
import numpy as np
import pybullet as p
import pybullet_data
import roboticstoolbox as rtb
from spatialmath import SE3, SO3
import time

import unittest
from scipy.spatial.transform import Rotation

from tools import *

'''
Notes

Quaternions
spatialmath uses scalar-first format: (w, x, y, z)
PyBullet and SciPy use scalar-last format: (x, y, z, w)
'''

SIM_FREQUENCY = 240 # Hz

def setEarthGravity():
    p.setGravity(0, 0, -9.81)

def configureEnvironment():
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # hide panes
    cameraSpecs = p.getDebugVisualizerCamera()
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=cameraSpecs[-4], cameraPitch=cameraSpecs[-3],
            cameraTargetPosition=cameraSpecs[-1]) # zoom in
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')
    setEarthGravity()

class Robot:
    def __init__(self):
        self.urdf = 'Manipulator/urdf/ManipulatorMod.urdf'
        p.connect(p.GUI)
        self.loadRobot()
        configureEnvironment()

    def __del__(self):
        p.disconnect()

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
            logId = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'downflop_bullet.mp4')
        else:
            p.setGravity(0, 0, 9.81)
            logId = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'upflop_bullet.mp4')
        self.setInitialConfiguration()

        time.sleep(5) # initial delay
        for i in range(1000):
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

        p.stopStateLogging(logId)
        if not gravityDown:
            setEarthGravity()

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

    def computedTorquePositionRegulator(self, targetJoint=None, targetCartesian=None, useCartesianOrientation=False,
            timeLimit=None):
        '''
        targetJoint is list of joint angles
        targetJoint is SE3 transform
        useCartesianOrientation indicates whether Cartesian orientation is required for inverse kinematics
        '''
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

        t = 0
        condition = False
        while not condition:
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
            t += 1 / SIM_FREQUENCY
            if timeLimit is not None and t > timeLimit:
                condition = True

    def computedTorqueTrajectoryFollower(self, trajectoryJoint=None, trajectoryCartesian=None,
            useCartesianOrientation=False):
        '''
        trajectoryJoint is joint space trajectory of type rtb.tools.trajectory.Trajectory
        trajectoryCartesian is a list of SE3 transforms
        useCartesianOrientation indicates whether Cartesian orientation is required for inverse kinematics
        '''
        if trajectoryJoint is None and trajectoryCartesian is not None:
            cartesian = True
            steps = len(trajectoryCartesian)
            # Set end effector at start
            maxIterations = 1000
            if useCartesianOrientation:
                quat = Rotation.from_matrix(trajectoryCartesian[0].R).as_quat()
                qPrev = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[0].t, quat,
                        maxNumIterations=maxIterations)
            else:
                qPrev = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[0].t,
                        maxNumIterations=maxIterations)
            maxIterations = 100
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

        time.sleep(5) # pause
        for i in range(steps):
            qC, qdC = self.getJointStates()
            if not cartesian:
                e = trajectoryJoint.s[i] - qC
                ed = trajectoryJoint.sd[i] - qdC
                qddT = trajectoryJoint.sdd[i]
            else:
                if useCartesianOrientation:
                    quat = Rotation.from_matrix(trajectoryCartesian[i].R).as_quat()
                    qT[:] = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[i].t, quat,
                            maxNumIterations=maxIterations)
                else:
                    qT[:] = p.calculateInverseKinematics(self.id, self.n - 1, trajectoryCartesian[i].t,
                            maxNumIterations=maxIterations)
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
            #print(self.getEndPose()[0])
            M = np.array(p.calculateMassMatrix(self.id, list(qC)))
            N = np.array(p.calculateInverseDynamics(self.id, list(qC), list(qdC), [0.] * self.n)) # sum of Coriolis and
            # gravity terms
            torque = M @ (qddT + Kv @ ed + Kp @ e) + N
            p.setJointMotorControlArray(self.id, list(range(self.n)), p.TORQUE_CONTROL, forces=torque)
            p.stepSimulation()
            time.sleep(1 / SIM_FREQUENCY)

def pidPositionRegulator():
    robot = Robot()
    angle = 20 * math.pi / 180
    robot.pidPositionRegulator(targetJoint=np.ones(7) * angle)

def pentagramTrajectoryFollower():
    robot = Robot()
    poses = getVertexPoses(getPentagramVertices(center=(0, .3), radius=0.1), normalAxis=0, planeOffset=.3)
    trajectory = getSegmentedTrajectory(poses)

    print(trajectory[0])
    
    robot.computedTorqueTrajectoryFollower(trajectoryCartesian=trajectory, useCartesianOrientation=True)

class Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot()

    def testFlop(self):
        self.robot.flop()
        self.robot.flop(False)

    def testComputedTorquePositionRegulator(self):
        # Joint
        angle = 20 * math.pi / 180
        self.robot.computedTorquePositionRegulator(targetJoint=np.ones(7) * angle, timeLimit=10)
        
        # Cartesian
        T = SE3(0, 0, 0.2) * SE3.Rx(math.pi / 2)
        self.robot.computedTorquePositionRegulator(targetCartesian=T, useCartesianOrientation=True, timeLimit=10)

    def testComputedTorqueTrajectoryFollower(self):
        # Joint
        initialAngle = 10 * math.pi / 180
        finalAngle = 45 * math.pi / 180
        q0 = np.ones(7) * initialAngle
        qf = np.ones(7) * finalAngle
        duration = 10
        t = np.linspace(0, duration, duration * SIM_FREQUENCY)
        trajectory = rtb.tools.trajectory.jtraj(q0, qf, t)
        self.robot.computedTorqueTrajectoryFollower(trajectoryJoint=trajectory)

        # Cartesian
        poses = getVertexPoses(getPentagramVertices(center=(0, .3), radius=0.1), normalAxis=0, planeOffset=.3)
        trajectory = getSegmentedTrajectory(poses)
        
        self.robot.computedTorqueTrajectoryFollower(trajectoryCartesian=trajectory, useCartesianOrientation=True)

if __name__ == '__main__':
    unittest.main()
