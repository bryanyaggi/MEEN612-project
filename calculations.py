#! /usr/bin/env python3

import math
import roboticstoolbox as rtb
import sympy as sym
from ch3 import linkTransform, linkTransformSym

def dh():
    b = 1/16
    l_1 = 1
    l_2 = 3
    l_3 = 5

    robot = rtb.DHRobot([
        rtb.RevoluteMDH(d=l_1 + b + l_2),
        rtb.RevoluteMDH(alpha=math.pi/2),
        rtb.RevoluteMDH(alpha=-math.pi/2, d=l_2 + b + l_3),
        rtb.RevoluteMDH(alpha=math.pi/2),
        rtb.RevoluteMDH(alpha=-math.pi/2, d=l_2 + b + l_3),
        rtb.RevoluteMDH(alpha=math.pi/2,),
        rtb.RevoluteMDH(alpha=-math.pi/2, d=l_2 + b + l_1)]
        )
    print(robot)
    robot.plot(q=[0, 0, 0, 0, 0, 0, 0], block=True)

    pyplot = rtb.backends.PyPlot()
    pyplot.launch()
    pyplot.add(robot)

def frames():
    panda = rtb.models.URDF.Panda()
    panda.plot([0, 0, 0, 0, 0, 0, 0], backend='pyplot', jointaxes=True, block=True)

def forwardKinematics():
    # Symbolic
    b, l_1, l_2, l_3 = sym.symbols('b l_1 l_2 l_3')
    q_1, q_2, q_3, q_4, q_5, q_6, q_7 = sym.symbols('q_1 q_2 q_3 q_4 q_5 q_6 q_7')
    
    T_sym = []
    T_sym.append(linkTransformSym(0,          0, l_1 + b + l_2, q_1))
    T_sym.append(linkTransformSym(0,  math.pi/2,             0, q_2))
    T_sym.append(linkTransformSym(0, -math.pi/2, l_2 + b + l_3, q_3))
    T_sym.append(linkTransformSym(0,  math.pi/2,             0, q_4))
    T_sym.append(linkTransformSym(0, -math.pi/2, l_2 + b + l_3, q_5))
    T_sym.append(linkTransformSym(0,  math.pi/2,             0, q_6))
    T_sym.append(linkTransformSym(0, -math.pi/2, l_1 + b + l_2, q_7))

    for i in range(len(T_sym)):
        print('T_%d%d' %(i, i + 1))
        print(T_sym[i])
        print(T_sym[i].subs([(b, 1/16), (l_1, 1), (l_2, 3), (l_3, 5)]))

if __name__ == '__main__':
    dh()
    frames()
    forwardKinematics()
