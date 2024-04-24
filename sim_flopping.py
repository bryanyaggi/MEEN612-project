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

def flopDown():
    robot = Robot()
    robot.flop()

def flopUp():
    robot = Robot()
    robot.flop(False)

if __name__ == '__main__':
    flopDown()
    #flopUp()
