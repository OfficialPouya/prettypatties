#!/usr/bin/env python
import numpy
from scipy.linalg import expm
from project_header import *


def skew_sym(a1,a2,a3):
    result = [[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]]
    return result

def screw_mat(w,v):
    screw = [[w[0][0],w[0][1],w[0][2],v[0]],[w[1][0],w[1][1],w[1][2],v[1]],[w[2][0],w[2][1],w[2][2],v[2]],[0,0,0,0]]
    return screw

def Get_MS():
    w1 = numpy.array(skew_sym(0,0,1))
    w2 = numpy.array(skew_sym(0,1,0))
    w3 = numpy.array(skew_sym(0,1,0))
    w4 = numpy.array(skew_sym(0,1,0))
    w5 = numpy.array(skew_sym(1,0,0))
    w6 = numpy.array(skew_sym(0,1,0))
    v1 = numpy.matmul(numpy.negative(w1),[[-150],[150],[10]])
    v2 = numpy.matmul(numpy.negative(w2),[[-150],[270],[162]])
    v3 = numpy.matmul(numpy.negative(w3),[[94],[270],[162]])
    v4 = numpy.matmul(numpy.negative(w4),[[307],[177],[162]])
    v5 = numpy.matmul(numpy.negative(w5),[[307],[260],[162]])
    v6 = numpy.matmul(numpy.negative(w6),[[390],[260],[162]])
    Z = numpy.zeros((4,4))
    S = numpy.array([Z,Z,Z,Z,Z,Z])
    S[0] = screw_mat(w1,v1)
    S[1] = screw_mat(w2,v2)
    S[2] = screw_mat(w3,v3)
    S[3] = screw_mat(w4,v4)
    S[4] = screw_mat(w5,v5)
    S[5] = screw_mat(w6,v6)
    M = [[0,-1,0, 390],[0,0,-1, 401],[1,0,0, 215.5],[0,0,0,1]]
    return M, S

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    return_value = [None, None, None, None, None, None]
    M,S = Get_MS()
    S[0] = S[0]*theta1
    S[1] = S[1]*theta2
    S[2] = S[2]*theta3
    S[3] = S[3]*theta4
    S[4] = S[4]*theta5
    S[5] = S[5]*theta6
    S1 = (expm(S[0]))
    S2 = (expm(S[1]))
    S3 = (expm(S[2]))
    S4 = (expm(S[3]))
    S5 = (expm(S[4]))
    S6 = (expm(S[5]))
    S12 = numpy.matmul(S1,S2)
    S123 = numpy.matmul(S12,S3)
    S1234 = numpy.matmul(S123,S4)
    S12345 = numpy.matmul(S1234,S5)
    St = numpy.matmul(S12345,S6)
    T = numpy.matmul(St,M)
    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value



def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    l1 = 152
    l2 = 120
    l3 = 244
    l4 = 93
    l5 = 213
    l6 = 83
    l7 = 83
    l8 = 82
    l9 = 53.5
    l10 = 59 
    xcen = xWgrip - 53.5*(numpy.cos(numpy.radians((yaw_WgripDegree))))+150
    ycen = yWgrip - 53.5*(numpy.sin(numpy.radians((yaw_WgripDegree))))-150
    zcen = zWgrip -10

    theta1 = numpy.arctan2(ycen,xcen)-(numpy.arcsin((l2-l4+l6)/numpy.sqrt(xcen**2+ycen**2)))

    x3end = xcen - (l7*numpy.cos(theta1)) + ((l6+27)*numpy.sin(theta1))
    y3end = ycen - ((l6+27)*numpy.cos(theta1)) - (l7*numpy.sin(theta1))
    z3end = zcen + l10 + l8

    csqared = ((x3end**2)+(y3end**2))+((z3end-l1)**2)
    c = numpy.sqrt(csqared)
    theta2 = (-1*numpy.arccos(((l5**2)-csqared-(l3**2))/(-2*l3*c))) - numpy.arcsin((z3end-l1)/c)
    theta3 = PI - numpy.arccos(((l3**2)+(l5**2)-csqared)/(2*l3*l5))
    theta4 = -(PI-(PI-theta3-(theta2)))
    theta5 = -PI/2
    theta6 = PI-(PI/2-theta1)-numpy.radians(yaw_WgripDegree)

    result = lab_fk(float(theta1), float(theta2), float(theta3), float(theta4), float(theta5), float(theta6))
    return result

    pass
