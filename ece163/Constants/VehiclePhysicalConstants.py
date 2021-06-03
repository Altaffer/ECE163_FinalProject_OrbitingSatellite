"""All the vehicle constants that define the various constants that are used in the simulation and control
   model. These are physical constants, mathematical constants, and the rest."""

import math
from ece163.Utilities import MatrixMath

# SIMULATION PROFILE
dT = 1/100	# Time step for simulation

# ** we will probably want to have a reference point for NED along path of the origin **
# ** reference point would have a pd of zero, so the altitude of the sat will be -pd **
InitialSpeed = 0.0	# [m/s]
InitialNorth = 0.0  # displacement to north [m]
InitialEast = 0.0   # displacement to east [m]
InitialDown = 0.0	 # [m], negative is above ground
InitialYawAngle = math.radians(0.0)	# initial heading angle [rad]


# ENVIORNMENT PROFILE
rho = 0.00  # [kg / m^3]
G = 6.67e-11  # Gravitational Constant [N*m^2/kg^2]
radius_e = 6.37101e6  # radius of the earth [m]
mass_e = 5.972e24  # mass of earth [kg]

# SPACECRAFT PROFILE FOR: insert_spacecraft_name_here
mass = 10.165  # [kg]
<<<<<<< HEAD
A = 0.066  # area of one (of two) solar arrays [m^2]
A_M = (2 * A) / mass  # area to mass ratio [kg/m^2]

Jxx = 0.0691  # [kg m^2]
Jyy = 0.1465  # [kg m^2]
Jzz = 0.1929  # [kg m^2]
Jxz = 0.7144  # [kg m^2]
=======
lengthX = 0.366  # [m]
lengthChassis = 0.2394  # [m]
lengthY = 3 * lengthChassis  # [m]
lengthZ = 0.1062  # [m]

Jxx = 0.0969  # [kg m^2]
Jyy = 0.1235  # [kg m^2]
Jzz = 0.1918  # [kg m^2]
Jxz = 0.00  # [kg m^2] ----------------- sad I will have to ask Max abt how to figure this out it is not given
>>>>>>> 56c256218827adf0d8bf93f70f57eddc22f8b4fc

Jbody = [[Jxx, 0., -Jxz], [0., Jyy, 0.], [-Jxz, 0., Jzz]]
Jdet = (Jxx * Jzz - Jxz ** 2)
JinvBody = MatrixMath.scalarMultiply(1. / Jdet, [[Jzz, 0., Jxz], [0., Jdet / Jyy, 0.], [Jxz, 0., Jxx]])

# THRUSTER profile - epulsion nano thruster
Thruster_min = 10e-6  # minimum thruster value [N]
Thruster_max = 330e6  # maximum thruster value [N]
C_thruster = Thruster_max - Thruster_min  # Constant relaying a thruster command to newtons generated

# ** because thruster control does not always operate on a range from 0 -> thruster_max, the following equation can be
# used: Fthrust = (C_thruster * control) + Thruster_min  -->  with corner cases being negative control (subtract Thruster
# min) and zero control (Fthrust = 0)

<<<<<<< HEAD
# REACTION WHEEL profile
Reaction_max = 0.1  # maximum reaction wheel tourque [N*m]
=======
# REACTION WHEEL profile - Blue Canyon Technologies - RWP015
Reaction_max = 0.004  # maximum reaction wheel tourque [N*m]
Reaction_momentum = 0.015  # momentum of reaction wheel, torque is the rate of change of this [N*m*s]
>>>>>>> 56c256218827adf0d8bf93f70f57eddc22f8b4fc
C_reaction = Reaction_max  # coefficient for getting reaction wheel tourque from control input

# ** the reaction wheel operates linearly unlike the thruster, so the torque is just a proportion (from controls) of the
# max torque it can apply

# DISTURBANCE CONSTANTS
#Distance vectors in ECEF
earthMoon = [[0], [0], [384e6]]     #Distance from the Earth to the Moon when direction overhead
earthSun = [[0], [0], [149e9]]      #Distance from the Earth to the Sun when direction overhead
earthJup = [[0], [0], [588e9]]      #Distance from the Earth to Jupiter when direction overhead

moonAcc = 7.3e-6  # accleration disturbance caused by moon (mean) [m/s^2]
sunAcc = 3.5e-6  # accleration disturbance caused by sun (mean) [m/s^2]
jupAcc = 5.2e-11  # accleration disturbance caused by jupiter (max) [m/s^2]
radiationAcc = 4.7e-6  # acceleration caused by radiation pressure [(m/s^2) * (m/A)]