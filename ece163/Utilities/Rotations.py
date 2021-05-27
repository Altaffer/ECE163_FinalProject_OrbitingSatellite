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

def dcm2Euler(DCM):
    """
    Extracts the Euler angles from the rotation matrix, in the form of yaw, pitch, roll corresponding to the [3,2,1] euler
    set. Note that euler angles have a singularity at pitch = +/- pi/2 in that roll and yaw become indistinguishable.
    Parameters:
    DCM – Rotation matrix [3 x 3]

    Returns:
    yaw, pitch, and roll [rad] in a 3 item array
    """
    euler_yaw_pitch_roll = [0,0,0] #create an array to hold yaw, pitch, roll

    #calculate yaw, checking for out of domain
    euler_yaw_pitch_roll[0] = math.atan2(DCM[0][1], DCM[0][0])

    # check for case where math gives a value slightly greater that 1 or slightly less than -1
    if DCM[0][2] - 1 > 0:
        DCM[0][2] = 1
    elif DCM[0][2] + 1 < 0:
        DCM[0][2] = -1

    # calculate pitch
    euler_yaw_pitch_roll[1] = -(math.asin(DCM[0][2]))

    #calculate roll
    euler_yaw_pitch_roll[2] = math.atan2(DCM[1][2], DCM[2][2])

    #return the euler array
    return euler_yaw_pitch_roll

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