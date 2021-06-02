import math
from ..Utilities import MatrixMath as mm

# ORBITAL FRAME
# defined as T, O, R axises where the basis vectors are
# T is the vector tangential to the circular orbit
# O is the vector normal to the orbital plane
# R is the radial vector from the center of the earth to the satellite's projection on the orbital plane

def orbitalFrameR(orbitVector, state):
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

