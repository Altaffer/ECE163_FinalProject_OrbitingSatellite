import math
from ..Utilities import MatrixMath as mm

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
    tangentialVector = mm.crossProduct(radialVector, orbitVector)

    oV_t = mm.transpose(orbitVector)
    rV_t = mm.transpose(radialVector)
    tV_t = mm.transpose(tangentialVector)

    R_ECI_to_Orbital = [tV_t, oV_t, rV_t]
    R_Orbital_to_ECI = mm.transpose(R_ECI_to_Orbital)

    return R_ECI_to_Orbital, R_Orbital_to_ECI

