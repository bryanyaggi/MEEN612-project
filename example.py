#!/usr/bin/env python3

import math
import numpy as np
import sympy as sym
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import swift

def run():
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
    run()
