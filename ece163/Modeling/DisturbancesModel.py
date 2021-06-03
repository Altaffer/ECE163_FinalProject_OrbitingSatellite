"""
Authors: Orbiting Satellite group

This file models the disturbances that the satellite will encounter in the orbit
"""
import math
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations as Rotations
from ..Constants import VehiclePhysicalConstants as VPC
import numpy as np
from pymap3d import ecef
from pymap3d import eci

def distanceFromMoon(state):
    """Calculates the vector between the Moon and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    #finding the difference between earthMoon and earthSatellite(state variables) to find distance
    moonSat = mm.subtract(VPC.earthMoon, [[state.pn], [state.pe], [state.pd]])
    #calculating the norm
    norm = mm.scalarDivide(np.linalg.norm(moonSat), moonSat)
    #rotation to body frame
    rot = mm.multiply(mm.transpose(Rotations.euler2DCM(state.yaw, state.pitch, state.roll)), norm)
    return rot

def distanceFromSun(state):
    """Calculates the vector between the Sun and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    # finding the difference between earthSun and earthSatellite(state variables) to find distance
    sunSat = mm.subtract(VPC.earthSun, [[state.pn], [state.pe], [state.pd]])
    # calculating the norm
    norm = mm.scalarDivide(np.linalg.norm(sunSat), sunSat)
    # rotation to body frame
    rot = mm.multiply(mm.transpose(Rotations.euler2DCM(state.yaw, state.pitch, state.roll)), norm)
    return rot

def distanceFromJupiter(state):
    """Calculates the vector between Jupiter and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    # finding the difference between earthSun and earthSatellite(state variables) to find distance
    jupSat = mm.subtract(VPC.earthJup, [[state.pn], [state.pe], [state.pd]])
    # calculating the norm
    norm = mm.scalarDivide(np.linalg.norm(jupSat), jupSat)
    # rotation to body frame
    rot = mm.multiply(mm.transpose(Rotations.euler2DCM(state.yaw, state.pitch, state.roll)), norm)
    return rot

def satSurfaceArea(state):
    """Calculates the surface area of the light from the sun hitting the satellite.
    """
    #area of the satellite
    A = VPC.lengthY * VPC.lengthX

    #orthoganal unit vector to solar array in body frame
    ortho_vector = [0,0,-1]

    #nomalized sun to sat array in body frame
    temp = distanceFromSun(state)
    sun_vector = [temp[0][0], temp[1][0], temp[2][0]]

    #solving for the unit vectors
    unit_ortho = ortho_vector / np.linalg.norm(ortho_vector)
    unit_sun = sun_vector / np.linalg.norm(sun_vector)

    # dot product between both vectors to find angle
    dot_product = np.dot(unit_ortho, unit_sun)

    # total surface area
    surf_area = math.fabs(A*dot_product)

    return surf_area