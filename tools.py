#! /usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, SO3

import unittest

def getPentagramVertices(center=(0, 0), radius=1):
    n = 5
    x = lambda theta : radius * math.cos(theta) + center[0]
    y = lambda theta : radius * math.sin(theta) + center[1]

    vertices = []
    indices = []
    for i in [3, 0, 2, 4, 1, 3]: # [bottom left, top, bottom right, left, right, bottom left]
        angle = i * 2 * math.pi / n + math.pi / 2 # top at pi/2
        vertices.append((x(angle), y(angle)))

    return vertices

def getVertexPoses(getVerticesFunc, normalAxis=2, planeOffset=1):
    vertices = getVerticesFunc

    poses = []
    for vertex in vertices:
        if normalAxis == 0:
            poses.append(SE3(planeOffset, vertex[0], vertex[1]) * SE3.Ry(math.pi / 2))
        elif normalAxis == 1:
            poses.append(SE3(vertex[0], planeOffset, vertex[1]) * SE3.Rx(math.pi / 2))
        elif normalAxis == 2:
            poses.append(SE3(vertex[0], vertex[1], planeOffset))
        else:
            raise ValueError('Invalid normal axis value')

    return poses

def getSegmentedTrajectory(poses, segmentDuration=5, stepFrequency=240):
    '''
    poses is list of SE3 transforms
    '''
    trajectory = []
    t = np.linspace(0, segmentDuration, segmentDuration * stepFrequency)
    for i in range(len(poses) - 1):
        trajectory.extend(rtb.tools.trajectory.ctraj(poses[i], poses[i + 1], t))

    return trajectory

class Test(unittest.TestCase):
    def testGetPentagramVertices(self):
        vertices = getPentagramVertices()

        fig, ax = plt.subplots()
        xs = [vertex[0] for vertex in vertices]
        ys = [vertex[1] for vertex in vertices]
        ax.plot(xs, ys)
        ax.scatter(xs, ys)
        ax.set_aspect('equal')
        plt.show()

    def testGetVertexPoses(self):
        poses = getVertexPoses(getPentagramVertices(), normalAxis=0)
        for pose in poses:
            print(pose)

    def testGetSegmentedTrajectory(self):
        poses = getVertexPoses(getPentagramVertices())
        trajectory = getSegmentedTrajectory(poses)
        for pose in trajectory:
            print(pose)

if __name__ == '__main__':
    unittest.main()
