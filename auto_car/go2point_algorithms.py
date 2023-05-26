import numpy as np
import time

currPoint = 0

def stack_p2p(targetPos, CurrPos, PHI, a_Max):
    global currPoint
    
    if currPoint == 0:
        currPoint = 1
    
    # Calculate car vectors at angle from desired path
    Vr = np.array([np.cos(PHI), np.sin(PHI)])  # Car direction vector
    Vt = targetPos[currPoint-1] - CurrPos  # Car direction vector
    
    crossVector = Vt[1]*Vr[0] - Vt[0]*Vr[1]  # cross(Vr, Vt)
    dotVector = np.dot(Vt, Vr) / (np.linalg.norm(Vt) * np.linalg.norm(Vr))
    
    # Calculate the desired angle change for the target point
    if abs(crossVector) < 0.001:  # Math singularity
        if dotVector == -1:  # Vectors align in reverse... singularity
            theta_t = np.pi/2
        else:  # Vectors align in the same direction
            theta_t = 0
    else:
        dir = np.sign(crossVector)
        theta_t = np.arccos(dotVector) * dir
    
    # Calculate distance to last point
    dist = np.linalg.norm(Vt)  # Distance to end point
    pathDist = 0  # Remaining points distance
    numPoints = targetPos.shape[0]
    
    # Sum of remaining points
    if currPoint < numPoints:
        for i in range(currPoint, numPoints):
            pathDist += np.sqrt(np.sum((targetPos[i, :] - targetPos[i-1, :])**2))
    
        # Pass to next point condition
        if abs(dist) < 0.05:
            currPoint += 1
    
        # Update remaining distance
        dist += pathDist
    
    # Set desired velocity
    if abs(dist) < 0.01:  # 1 cm stop condition, happens only if reached last point
        V_Forward = 0
        theta_t = 0
        En = 0
    else:
        V_Forward = np.sqrt(2 * a_Max * dist)
        En = 1
    
    return V_Forward, theta_t, dist, En


def p2p(X_target, Y_target, POS_X, POS_Y, PHI, a_Max):
    # calculate car vectors at angle from desired path
    Vr = np.array([np.cos(PHI), np.sin(PHI)])  # car direction vector
    Vt = np.array([X_target - POS_X, Y_target - POS_Y])  # car direction vector

    crossVector = Vt[1] * Vr[0] - Vt[0] * Vr[1]  # cross(Vr, Vt)
    dotVector = np.dot(Vt, Vr) / (np.linalg.norm(Vt) * np.linalg.norm(Vr))

    # calculate the desired angle change for the target point
    if abs(crossVector) < 0.001:  # Math singularity
        if dotVector == -1:  # vectors align in reverse... singularity
            theta_t = np.pi / 2
        else:  # Vectors align in the same direction
            theta_t = 0
    else:
        dir = np.sign(crossVector)
        theta_t = np.arccos(dotVector) * dir

    # set desired velocity
    dist = np.linalg.norm(Vt)  # distance to end point
    if abs(dist) < 0.01:  # 1 cm stop condition
        V_Forward = 0
        theta_t = 0
        En = 0
    else:
        V_Forward = np.sqrt(2 * a_Max * dist)
        En = 1

    return V_Forward, theta_t, dist, En

