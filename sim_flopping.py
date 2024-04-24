#!/usr/bin/env python3

import math
import numpy as np
import os
import roboticstoolbox as rtb
from roboticstoolbox.robot.Robot import Robot
import spatialmath as sm
import spatialgeometry as sg
import swift

class Manipulator(Robot):
    def __init__(self):
        path = os.getcwd()
        path += '/Manipulator/urdf/Manipulator.urdf'
        links, name, urdf_string, urdf_filepath = self.URDF_read(path)
        super().__init__(links, name=name, urdf_string=urdf_string,
                urdf_filepath=urdf_filepath)
        self.manufacturer = 'Bryan Yaggi'

        self.qz = np.zeros(7)
        self.addconfiguration_attr('qz', self.qz)

def flop(gravityDown=True):
    robot = Manipulator()

    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot)

def test():
    #robot = rtb.models.URDF.UR10()
    #robot = rtb.models.URDF.Puma560()
    robot = rtb.models.DH.Puma560()
    q = np.zeros(6)
    M = robot.inertia(q)
    print(M)

def control():
    env = swift.Swift()
    env.launch(realtime=True)

    robot = rtb.models.Panda()
    robot.q = robot.qr # pre-defined 'ready' configuration

    Tep = robot.fkine(robot.q) * sm.SE3.Trans(0.2, 0.2, 0.45) # target end effector pose
    axes = sg.Axes(length=0.1, pose=Tep)
    env.add(axes)

    arrived = False
    dt = 0.05
    env.add(robot)

    while not arrived:
        v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, gain=1, threshold=0.01) # v is desired velocity
        robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v # qd is joint velocity vector
        env.step(dt)
    
if __name__ == '__main__':
    #flop()
    test()
