import numpy as np
from const import * 

def jacobianIK(O, endPos, startPos): 
    while(np.abs(startPos - endPos) > 0.001): 
        dO = getDeltaOrientation(endPos, startPos)
        O += dO * 0.001
        startPos = FK(O)

def getDeltaOrientation(endPos, startPos): 
    Jt = getJacobianTranspose(endPos, startPos)
    V = endPos - startPos
    dO = Jt @ V; 
    return dO

def getJacobianTranspose(endPos, startPos):
    xRotation = np.array([1, 0, 0])
    yRotation = np.array([0, 1 ,0])
    # zRotation = np.array([0, 0, 1])
    
    J_A = np.cross(xRotation, endPos-startPos)
    J_B = np.cross(yRotation, endPos-startPos)
    J_C = np.cross(yRotation, endPos-startPos)

    J = J = np.vstack((J_A, J_B, J_C))
    return J.transpose()

def FK(O): 
    convertToRad = lambda x: x*(np.pi/180)

    # Finding alpha 4 and 5 (angles between the arm and forearm)
    alpha_4 = 180 - O[1] - 90 
    alpha_5 = O[2] - alpha_4

    # Finding the height of the arm minus the height of the base (d3)
    d3 = ARM_LENGHT*np.sin(convertToRad(O[1]))

    # Finding the forearm height d6 
    d6 = FOREARM_LENGHT*np.cos(convertToRad(alpha_5))

    #
    return 0