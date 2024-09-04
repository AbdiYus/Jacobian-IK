import numpy as np
from const import * 

def jacobianIK(O, endPos, startPos, EPS): 
    limit = 0
    while np.linalg.norm(startPos - endPos) > EPS and limit < 100: 
        dO = getDeltaOrientation(endPos, startPos)
        O += dO * 0.001
        startPos = FK(O)
        limit += 1
        print(f"Startposition: {startPos}, and endpos: {endPos}")
        if limit == 99:
            print("99 done")
    return O

def getDeltaOrientation(endPos, startPos): 
    Jt = getJacobianTranspose(endPos, startPos)
    V = endPos - startPos
    dO = Jt @ V 
    return dO

def getJacobianTranspose(endPos, startPos):
    xRotation = np.array([1, 0, 0])
    yRotation = np.array([0, 1 ,0])
    zRotation = np.array([0, 0, 1])
    
    J_A = np.cross(zRotation, endPos-startPos)
    J_B = np.cross(yRotation, endPos-startPos)
    J_C = np.cross(yRotation, endPos-startPos)

    J = np.vstack((J_A, J_B, J_C))
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

    # Finding z 
    z = BASE_HEIGHT + d3 - d6

    # Finding the hypotanus of the arm top-view (x and y)
    d4 = ARM_LENGHT*np.cos(convertToRad(O[1]))
    d5 = FOREARM_LENGHT*np.sin(convertToRad(alpha_5))
    d1 = d4 + d5

    # Finding x and y 
    y = d1*np.sin(convertToRad(O[0]))
    x = d1*np.cos(convertToRad(O[0]))
    
    return np.array([x, y, z])


################################################################################
# Implementing gradient decent 

def distanceFromTarget(target, O):
    currentPos = FK(O) 
    return np.linalg.norm(currentPos - target)

def partialGradient(target, O, i, delta=1e-3):
    angle = O[i] 
    f_x = distanceFromTarget(target, O)
    O[i] += delta 
    f_x_plus_d = distanceFromTarget(target, O)
    gradient = (f_x_plus_d - f_x) / delta 
    O[i] = angle 
    return gradient 

def IK(target, O, learning=0.1, threshold=0.1, max_iterations=1000):
    for iteration in range(max_iterations):
        if distanceFromTarget(target, O) < threshold:
            return
        for i in range(2, -1, -1):  # amount of joints of the robot
            gradient = partialGradient(target, O, i)
            O[i] -= learning * gradient
            print(f"Iteration {iteration}, Joint {i}: Gradient = {gradient}, Angle Before = {O[i] + learning * gradient}")
            print(f"Angle After = {O[i]}")
            if distanceFromTarget(target, O) < threshold:
                return
    print(f"result: {O} and position:{FK(O)}")
