"""
Authors: Orbiting Satellite group

This file models the disturbances that the satellite will encounter in the orbit
"""

from ..Utilities import MatrixMath as mm
from ..Constants import VehiclePhysicalConstants as VPC
from pymap3d import ecef
from pymap3d import eci

def distanceFromMoon(p_north, p_east, p_down, lat, lon, height):
    """Calculates the vector between the Moon and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    # converting inputed NED positions to ecef coordinates
    xp, yp, zp = ecef.enu2ecef(p_east, p_north, -p_down, lat, lon, height, ell=None, deg=True)
    #finding the difference between earthMoon and earthSatellite(state variables) to find distance
    moonSat = mm.add(VPC.earthMoon, [[xp], [yp], [zp]])
    #calculating the norm
    norm = math.sqrt((moonSat[0][0] ** 2) + (moonSat[1][0] ** 2) + (moonSat[2][0] ** 2))
    return norm

def distanceFromSun(p_north, p_east, p_down, lat, lon, height):
    """Calculates the vector between the Sun and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    #converting inputed NED positions to ecef coordinates
    xp, yp, zp = ecef.enu2ecef(p_east, p_north, -p_down, lat, lon, height, ell=None, deg=True)
    # finding the difference between earthSun and earthSatellite(state variables) to find distance
    sunSat = mm.add(VPC.earthSun, [[xp], [yp], [zp]])
    # calculating the norm
    norm = math.sqrt((sunSat[0][0] ** 2) + (sunSat[1][0] ** 2) + (sunSat[2][0] ** 2))
    return norm

def distanceFromJupiter(p_north, p_east, p_down, lat, lon, height):
    """Calculates the vector between Jupiter and the satellite by inputting the NED positions of the satellite
    throughout orbit using Earth distances as reference points. Returns normalized vector.
    """
    #converting inputed NED positions to ecef coordinates
    xp, yp, zp = ecef.enu2ecef(p_east, p_north, -p_down, lat, lon, height, ell=None, deg=True)
    # finding the difference between earthSun and earthSatellite(state variables) to find distance
    jupSat = mm.add(VPC.earthJup, [[xp], [yp], [zp]])
    # calculating the norm
    norm = math.sqrt((jupSat[0][0] ** 2) + (jupSat[1][0] ** 2) + (jupSat[2][0] ** 2))
    return norm

def satSurfaceArea():
    """Calculates the surface area of the light from the sun hitting the satellite. """
