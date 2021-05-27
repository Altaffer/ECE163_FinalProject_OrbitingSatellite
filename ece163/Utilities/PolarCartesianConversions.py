"""
Authors: Orbiting Satellite Group

This file serves as a utility which will convert our NED system to Lat/Long/Alt and vice versa
"""

def cartesianToPolar(north, east, down):
    """
    This function converts cartesian NED units to corresponding Lat/Long/Alt coordinates.

    ** may have to normalize NED coordinates? **

    parameters
    north - meters north of reference [m]
    east - meters south of reference [m]
    down - meters below reference [m]

    returns
    lat, long, lat - coordinates of the spacecraft
    """
    return

def polarToCartesian(lat, long, alt):
    """

    parameters
    lat - latitude coordinate of spacecraft [degrees]
    long - longitude coordinate of spacecraft [degrees]
    alt - altitude coordinate of spacecraft [degrees]

    returns
    north, east, down - meters north, east, down from reference
    """
    return