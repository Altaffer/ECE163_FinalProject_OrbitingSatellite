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
    moonSat = mm.subtract(VPC.earthMoon, [[state.pn], [state.pe], [-state.pd]])
    #calculating the norm
    norm = math.sqrt((moonSat[0][0] ** 2) + (moonSat[1][0] ** 2) + (moonSat[2][0] ** 2))
    return mm.scalarDivide(moonSat, norm)

def distanceFromSun(state):
    """Calculates the vector between the Sun and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    # finding the difference between earthSun and earthSatellite(state variables) to find distance
    sunSat = mm.subtract(VPC.earthSun, [[state.pn], [state.pe], [-state.pd]])
    # calculating the norm
    norm = math.sqrt((sunSat[0][0] ** 2) + (sunSat[1][0] ** 2) + (sunSat[2][0] ** 2))
    return mm.scalarDivide(sunSat, norm)

def distanceFromJupiter(state):
    """Calculates the vector between Jupiter and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    # finding the difference between earthSun and earthSatellite(state variables) to find distance
    jupSat = mm.add(VPC.earthJup, [[state.pn], [state.pe], [-state.pd]])
    # calculating the norm
    norm = math.sqrt((jupSat[0][0] ** 2) + (jupSat[1][0] ** 2) + (jupSat[2][0] ** 2))
    return mm.scalarDivide(jupSat, norm)

def satSurfaceArea(state):
    """Calculates the surface area of the light from the sun hitting the satellite.
    """
    #area of the satellite
    A = VPC.lengthY * VPC.lengthX
    # Identifying the vector the satellite is pointing
    vector =  mm.multiply([[0],[0],[-1]], mm.transpose(Rotations.euler2DCM(state.yaw, state.pitch, state.roll)))
    unit_vector = mm.scalarDivide(vector, np.linalg.norm(vector))
    # dot product between both vectors
    dot_product = np.dot(distanceFromSun(state), unit_vector)
    # total surface area
    surf_area = A * math.cos(dot_product)
    return surf_area