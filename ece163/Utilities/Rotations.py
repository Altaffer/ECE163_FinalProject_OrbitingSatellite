"""
Author: Richard Owens (rivowens@ucsc.edu)
This file contains useful mathmatical tools for calculating rotations using euler angles
"""

import math
from ..Utilities import MatrixMath as mm
#import MatrixMath as mm

def s(var):
    """
    This is a math helper function that simply shortens the math.sin(var) function to s(var)

    Parameters:
    var - the variable to perform the trig operation on

    Returns:
    the trig sin operation of the input
    """
    value = math.sin(var) #the value to be returned
    return value

def c(var):
    """
    This is a math helper function that simply shortens the math.cos(var) function to c(var)

    Parameters:
    var - the variable to perform the trig operation on

    Returns:
    the trig cos operation of the input
    """
    value = math.cos(var) #the value to be returned
    return value

def dcm2Euler(dcm):
    """
    reads a 3x3 rotation matrix and calculates the euler angles
    returns a list [yaw, pitch, roll]
    """
    # yaw calculations
    yaw = math.atan2(dcm[0][1], dcm[0][0])

    # pitch calculations
    # the min, max ensures that the value remains within [-1,1]
    boundedCell = min(1,max(-1,dcm[0][2]));
    pitch = -math.asin(boundedCell)

    # roll calculations
    roll = math.atan2(dcm[1][2], dcm[2][2])

    return yaw, pitch, roll

def euler2DCM(yaw, pitch, roll):
    """Create the direction cosine matrix, R, from the euler angles (assumed to be in radians). Angles are yaw,pitch,roll
    passed as individual arguments and in the form corresponding to the [3-2-1] Euler set. The DCM goes from inertial [I]
    vectors into the body frame [B].

    Parameters:
    yaw – rotation about inertial down (rad)
    pitch – rotation about intermediate y-axis (rad)
    roll – rotation about body x-axis (rad)

    Returns
    Direction Cosine Matrix (DCM) [3 x 3]
    """
    #create dcm with corresponding rotation vector
    dcm = [[(c(pitch) * c(yaw)), (c(pitch) * s(yaw)), -(s(pitch))],
           [((s(roll) * s(pitch) * c(yaw)) - (c(roll) * s(yaw))), ((s(roll) * s(pitch) * s(yaw)) + (c(roll) * c(yaw))), (s(roll) * c(pitch))],
           [((c(roll) * s(pitch) * c(yaw)) + (s(roll) * s(yaw))), ((c(roll) * s(pitch) * s(yaw)) - (s(roll) * c(yaw))), (c(roll) * c(pitch))]]
    return dcm

def ned2enu(points):
    """
    Function changes coordinates from North-East-Down (NED) to East-North-Up (ENU). This is required because while all of the
    dynamics and rotations are defined in NED, the graphics functions use ENU

    Parameters:
    points – matrix of point in NED [n x 3]

    Returns:
    same set of [n x 3] points in ENU coordinates
    """
    #create a variable to hold the enu return
    enu = []

    #define the ned -> enu rotation matrix
    r_ned2enu = [[0,1,0],
                 [1,0,0],
                 [0,0,-1]]

    #multiply the matrices and set value to eny
    enu = mm.multiply(points, r_ned2enu)

    return enu