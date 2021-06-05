import math
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations

from ..Containers import States

# ORBITAL FRAME
# defined as T, O, R axises where the basis vectors are
# T is the vector tangential to the circular orbit
# O is the vector normal to the orbital plane
# R is the radial vector from the center of the earth to the satellite's projection on the orbital plane

def orbitalFrameR(orbitVector, state:States.vehicleState):
    # The first component of the orbital frame
    # vector normal to the orbital plane
    orbitVector = mm.vectorNorm(orbitVector)

    # generating the vector along the orbital plane
    # that faces towards the center of the earth
    positionVector = [[state.pn], [state.pe], [state.pd]]
    # the projection of the satellite vector onto the orbital plane
    planeRadialVector = mm.vectorRejection(positionVector, orbitVector)
    # the normalized vector, second component of the orbital frame
    radialVector = mm.vectorNorm(planeRadialVector)

    # cross product of the other two basis vectors of the orbital frame
    tangentialVector = mm.crossProduct(orbitVector, radialVector)

    oV_t = mm.transpose(orbitVector)
    rV_t = mm.transpose(radialVector)
    tV_t = mm.transpose(tangentialVector)

    R_ECI_to_Orbital = [tV_t[0], oV_t[0], rV_t[0]]
    R_Orbital_to_ECI = mm.transpose(R_ECI_to_Orbital)

    return R_ECI_to_Orbital, R_Orbital_to_ECI

def getBodyOrbitalRotationMatrices(R_e2o, R_o2e, vehicleState:States.vehicleState):
    # Getting Rotation Matrix from body 2 orbital frame
    R_b2e = mm.transpose(vehicleState.R) # body to inertial is equivalent to body to ECI
    R_b2o = mm.multiply(R_e2o, R_b2e)
    R_o2b = mm.transpose(R_b2o)

    return R_b2o, R_o2b

def getOrbitalAxisVals(R_e2o, R_o2e, vehicleState:States.vehicleState):
    # Getting Rotation Matrix from body 2 orbital frame
    R_b2e = mm.transpose(vehicleState.R) # body to inertial is equivalent to body to ECI
    R_b2o = mm.multiply(R_e2o, R_b2e)
    R_o2b = mm.transpose(R_b2o)

    # Getting Position in Orbital Frame
    ECI_Pos = [[vehicleState.pn], [vehicleState.pe], [vehicleState.pd]]
    ORB_Pos = mm.multiply(R_e2o, ECI_Pos)

    # Getting Velocity in Orbital Frame
    Body_Vel = [[vehicleState.u], [vehicleState.v], [vehicleState.w]]
    ECI_Vel = mm.multiply(R_b2e, Body_Vel)
    ORB_Vel = mm.multiply(R_e2o, ECI_Vel)

    return ORB_Pos, ORB_Vel

def getOrbitalAngularVals(R_e2o, R_o2e, vehicleState:States.vehicleState):
    # Getting Rotation Matrix from body 2 orbital frame
    R_b2e = mm.transpose(vehicleState.R) # body to inertial is equivalent to body to ECI
    R_b2o = mm.multiply(R_e2o, R_b2e)
    R_o2b = mm.transpose(R_b2o)

    # Getting euler angle misalignment between
    # orbital frame and body frame
    yaw, pitch, roll = Rotations.dcm2Euler(R_o2b)

    # body rotation rates
    p = vehicleState.p
    q = vehicleState.q
    r = vehicleState.r
    
    # Getting Yaw, Pitch, Roll dot in orbital frame
    pqr = mm.transpose([[p, q, r]])
    # beard 3.3
    weird_matrix = [[1, math.sin(roll)*math.tan(pitch), math.cos(roll)*math.tan(pitch)],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll)/math.cos(pitch), math.cos(roll)/math.cos(pitch)]]
    rollpitchyaw_dot = mm.multiply(weird_matrix, pqr)
    rollDot, pitchDot, yawDot = mm.transpose(rollpitchyaw_dot)[0]

    return yaw, pitch, roll, yawDot, pitchDot, rollDot