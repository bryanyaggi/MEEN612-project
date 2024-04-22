#!/usr/bin/env python3

import math
import numpy as np
#import sympy as sym
import roboticstoolbox as rtb
from roboticstoolbox.robot.Robot import Robot
import spatialmath as sm
import spatialgeometry as sg
import swift

class Manipulator(Robot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read(
                '/home/yaggi/courses/MEEN612-project/manipulator_sw/manipulator/urdf/ManipulatorRel.urdf')
        super().__init__(
                links,
                name=name,
                urdf_string=urdf_string,
                urdf_filepath=urdf_filepath)
        self.manufacturer = 'Bryan Yaggi'

        self.qz = np.zeros(7)
        self.addconfiguration_attr('qz', self.qz)

def run():
    env = swift.Swift()
    env.launch(realtime=True)

    robot = Manipulator()
    print(robot)

    #robot = rtb.models.Panda()
    #robot = rtb.models.DH.Panda()
    #robot.q = robot.qr # pre-defined 'ready' configuration
    #print(type(robot.q))
    
    robot.q = robot.qz
    robot.qd = np.zeros(7)
    #robot.plot(robot.q, backend='swift', block=True)
    env.add(robot)

    # Test dynamics
    #M = robot.inertia(robot.q)
    #print(M)

    '''
    Tep = robot.fkine(robot.q) * sm.SE3.Trans(0.2, 0.2, 0.45) # target end effector pose
    axes = sg.Axes(length=0.1, pose=Tep)
    env.add(axes)

    arrived = False
    dt = 0.05

    while not arrived:
        v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, gain=1, threshold=0.01) # v is desired velocity
        robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v # qd is joint velocity vector
        env.step(dt)
    '''
    
    robot = robot.nofriction()
    robot.dynamics()

    #d = robot.fdyn(5, robot.qz, lambda r, t, q, qd: np.zeros((r.n,)), dt=0.05)
    qdd = robot.accel(np.zeros((7,)), np.zeros((7,)), np.zeros((7,)))
    print(qdd)
    d = robot.fdyn(5, [0, 0, 0, 0, 0, 0, 0], progress=True)
    print(d.q)
    
if __name__ == '__main__':
    run()
