#!/usr/bin/env python3

import math
import numpy as np
import sympy as sym
import matplotlib.pyplot as plt

import unittest

def linkTransform(length, twist, offset, angle):
    '''
    a: length
    alpha: twist
    d: offset
    theta: angle
    '''
    S_X = np.zeros((4, 4))
    S_X[:3, :3] = np.array([
        [1, 0, 0],
        [0, math.cos(twist), -math.sin(twist)],
        [0, math.sin(twist), math.cos(twist)]])
    S_X[0, 3] = length
    S_X[3, 3] = 1
    
    S_Z = np.zeros((4, 4))
    S_Z[:3, :3] = np.array([
        [math.cos(angle), -math.sin(angle), 0],
        [math.sin(angle), math.cos(angle), 0],
        [0, 0, 1]])
    S_Z[2, 3] = offset
    S_Z[3, 3] = 1

    T = S_X @ S_Z

    return T

def linkTransformSym(length, twist, offset, angle):
    S_X = sym.Matrix([
        [1, 0, 0, length],
        [0, sym.cos(twist), -sym.sin(twist), 0],
        [0, sym.sin(twist), sym.cos(twist), 0],
        [0, 0, 0, 1]])

    S_Z = sym.Matrix([
        [sym.cos(angle), -sym.sin(angle), 0, 0],
        [sym.sin(angle), sym.cos(angle), 0, 0],
        [0, 0, 1, offset],
        [0, 0, 0, 1]])

    T = S_X @ S_Z

    return T

def ex3_6():
    # Assume values for all parameters
    T_01 = linkTransform(0, 0, 0, 30 * math.pi / 180)
    T_12 = linkTransform(0, 90 * math.pi/ 180, 2, 0)
    T_23 = linkTransform(0, 0, 4, 30 * math.pi / 180)

    print(T_01)
    print(T_12)
    print(T_23)

    # Keep symbolic parameters
    alpha_0, a_0, d_1, theta_1 = sym.symbols('alpha_0 a_0 d_1 theta_1')
    T_01_sym = linkTransformSym(a_0, alpha_0, d_1, theta_1)
    T_01_sym_simp = T_01_sym.subs([(a_0, 0), (alpha_0, 0), (d_1, 0)])
    alpha_1, a_1, d_2, theta_2 = sym.symbols('alpha_1 a_1 d_2 theta_2')
    T_12_sym = linkTransformSym(a_1, alpha_1, d_2, theta_2)
    T_12_sym_simp = T_12_sym.subs([(a_1, 0), (alpha_1, 90 * sym.pi / 180), (theta_2, 0)])
    alpha_2, a_2, d_3, theta_3 = sym.symbols('alpha_2 a_2 d_3 theta_3')
    T_23_sym = linkTransformSym(a_2, alpha_2, d_3, theta_3)
    T_23_sym_simp = T_23_sym.subs([(a_2, 0), (alpha_2, 0)])

    print(T_01_sym_simp)
    print(T_12_sym_simp)
    print(T_23_sym_simp)

    T_03 = T_01_sym_simp @ T_12_sym_simp @ T_23_sym_simp
    print(T_03)

def puma560():
    alpha_0, a_0, d_1, theta_1 = sym.symbols('alpha_0 a_0 d_1 theta_1')
    T_01 = linkTransformSym(a_0, alpha_0, d_1, theta_1).subs([(alpha_0, 0), (a_0, 0), (d_1, 0)])
    print(T_01)
    alpha_1, a_1, d_2, theta_2 = sym.symbols('alpha_1 a_1 d_2 theta_2')
    T_12 = linkTransformSym(a_1, alpha_1, d_2, theta_2).subs([(alpha_1, -90 * sym.pi / 180), (a_1, 0), (d_2, 0)])
    print(T_12)
    alpha_2, a_2, d_3, theta_3 = sym.symbols('alpha_2 a_2 d_3 theta_3')
    T_23 = linkTransformSym(a_2, alpha_2, d_3, theta_3).subs([(alpha_2, 0)])
    print(T_23)
    alpha_3, a_3, d_4, theta_4 = sym.symbols('alpha_3 a_3 d_4 theta_4')
    T_34 = linkTransformSym(a_3, alpha_3, d_4, theta_4).subs([(alpha_3, -90 * sym.pi / 180)])
    print(T_34)
    alpha_4, a_4, d_5, theta_5 = sym.symbols('alpha_4 a_4 d_5 theta_5')
    T_45 = linkTransformSym(a_4, alpha_4, d_5, theta_5).subs([(alpha_4, 90 * sym.pi / 180), (a_4, 0), (d_5, 0)])
    print(T_45)
    alpha_5, a_5, d_6, theta_6 = sym.symbols('alpha_5 a_5 d_6 theta_6')
    T_56 = linkTransformSym(a_5, alpha_5, d_6, theta_6).subs([(alpha_5, -90 * sym.pi / 180), (a_5, 0), (d_6, 0)])
    print(T_56)

    T_06 = T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56
    print(T_06)

class Test(unittest.TestCase):
    def testEx3_6(self):
        ex3_6()

    def testPuma560(self):
        puma560()

if __name__ == '__main__':
    unittest.main()
