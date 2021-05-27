"""All the vehicle constants that define the various constants that are used in the simulation and control
   model. These are physical constants, mathematical constants, and the rest."""

import math
from ece163.Utilities import MatrixMath

# SIMULATION PROFILE
dT = 1/100	# Time step for simulation

# ** we will probably want to have a reference point for NED along path of the origin **
InitialSpeed = 0	# [m/s]
InitialHeight = 0	# [m]
InitialNorthPosition = 0.0	# displacement to north [m]
InitialEastPosition = 0.0	# displacement to east [m]
InitialDownPosition = 0	# [m], negative is above ground
InitialYawAngle = math.radians(0.0)	# initial heading angle [rad]


# ENVIORNMENT PROFILE
rho = 0.00  # [kg / m^3]
G = 6.67e-11  # Gravitational Constant [N*m^2/kg^2]
radius_e = 6.37101e6  # radius of the earth [m]
mass_e = 5.972e24  # mass of earth [kg]

# SPACECRAFT PROFILE FOR: insert_spacecraft_name_here
mass = 0.00  # [kg]
A = 0.00  # area of one (of two) solar arrays [m^2]
A_M = (2 * A) / mass  # area to mass ratio [kg/m^2]

Jxx = 0.00  # [kg m^2]
Jyy = 0.00  # [kg m^2]
Jzz = 0.00  # [kg m^2]
Jxz = 0.00  # [kg m^2]

Jbody = [[Jxx, 0., -Jxz], [0., Jyy, 0.], [-Jxz, 0., Jzz]]
Jdet = (Jxx * Jzz - Jxz ** 2)
JinvBody = MatrixMath.scalarMultiply(1. / Jdet, [[Jzz, 0., Jxz], [0., Jdet / Jyy, 0.], [Jxz, 0., Jxx]])
