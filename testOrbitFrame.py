from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import OrbitalFrame as of
from ece163.Utilities import Rotations
from ece163.Containers import States
import math

isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-9)
def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)


######################################################################################################################################

# Testing with no orbital offset
orbit = [[0],[0],[-1]] # defines a counter clockwise orbit around the equator
position_inertial = [[100],[0],[0]] # position 100 units from center of earth
position = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0])
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, position)

position_of = mm.multiply(R_EtoO, position_inertial)
position_of_exp = [[0],[0],[100]] # expected that the R component is equal to the distance from center

res = compareVectors(position_of, position_of_exp)
assert(res)

identity = [[1,0,0],[0,1,0],[0,0,1]]
RRt = mm.multiply(R_EtoO, R_OtoE)
res = compareVectors(identity, RRt)
assert(res)

# testing that the euler angles of the orbital frame with respect to ECI are as expected
yaw, pitch, roll = Rotations.dcm2Euler(R_EtoO)
assert(isclose(yaw, math.radians(-90)))
assert(isclose(pitch, math.radians(0)))
assert(isclose(roll, math.radians(-90)))
# testing that the euler angles of ECI with respect to orbital frame are as expecter
# yaw, pitch, roll = Rotations.dcm2Euler(R_OtoE)
# print("yaw", yaw)
# print("pitch", pitch)
# print("roll", roll)
# assert(isclose(yaw, math.radians(90)))
# assert(isclose(pitch, math.radians(-90)))
# assert(isclose(roll, math.radians(0)))


# testing that orbitalVectors are in the expected direction
Torbital = [[1],[0],[0]]
Oorbital = [[0],[1],[0]]
Rorbital = [[0],[0],[1]]

Teci = mm.multiply(R_OtoE, Torbital)
Oeci = mm.multiply(R_OtoE, Oorbital)
Reci = mm.multiply(R_OtoE, Rorbital)

Teci_exp = [[0],[-1],[0]]
Oeci_exp = [[0],[0],[-1]]
Reci_exp = [[1],[0],[0]]

res = compareVectors(Teci, Teci_exp)
assert(res)
res = compareVectors(Oeci, Oeci_exp)
assert(res)
res = compareVectors(Reci, Reci_exp)
assert(res)

# testing getBodyOrbitalRots
R_b2o, R_o2b = of.getBodyOrbitalRots(R_EtoO, R_OtoE, position)
Torbital = [[1],[0],[0]]
Oorbital = [[0],[1],[0]]
Rorbital = [[0],[0],[1]]
Xbody = [[1],[0],[0]]
Ybody = [[0],[1],[0]]
Zbody = [[0],[0],[1]]

Tbody = mm.multiply(R_o2b, Torbital)
Obody = mm.multiply(R_o2b, Oorbital)
Rbody = mm.multiply(R_o2b, Rorbital)

Xorbital = mm.multiply(R_b2o, Xbody)
Yorbital = mm.multiply(R_b2o, Ybody)
Zorbital = mm.multiply(R_b2o, Zbody)

Tbody_exp = [[0],[-1],[0]]
Obody_exp = [[0],[0],[-1]]
Rbody_exp = [[1],[0],[0]]

Xorbital_exp = [[0],[0],[1]]
Yorbital_exp = [[-1],[0],[0]]
Zorbital_exp = [[0],[-1],[0]]

res = compareVectors(Tbody, Tbody_exp)
assert(res)
res = compareVectors(Obody, Obody_exp)
assert(res)
res = compareVectors(Rbody, Rbody_exp)
assert(res)
res = compareVectors(Xorbital, Xorbital_exp)
assert(res)
res = compareVectors(Yorbital, Yorbital_exp)
assert(res)
res = compareVectors(Zorbital, Zorbital_exp)
assert(res)

# Testing with an offset from the orbit
position_inertial = [[100],[0],[-20]] # position 100 units from center of earth
position = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0])
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, position)

position_of = mm.multiply(R_EtoO, position_inertial)
position_of_exp = [[0],[20],[100]] # expected that the R component is equal to the distance from center
                                   # O is the vertical offset

res = compareVectors(position_of, position_of_exp)
assert(res)

identity = [[1,0,0],[0,1,0],[0,0,1]]
RRt = mm.multiply(R_EtoO, R_OtoE)
res = compareVectors(identity, RRt)
assert(res)

# Testing with an orbit not around the equator
orbit = [[0],[1],[-1]] # defines a counter clockwise orbit around the equator, then rotated 45 degrees about inertial X
position_inertial = [[100],[0],[0]] # position 100 units from center of earth
position = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0])
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, position)

position_of = mm.multiply(R_EtoO, position_inertial)
position_of_exp = [[0],[0],[100]] # expected that the R component is equal to the distance from center
                                   # O is the vertical offset

res = compareVectors(position_of, position_of_exp)
assert(res)

identity = [[1,0,0],[0,1,0],[0,0,1]]
RRt = mm.multiply(R_EtoO, R_OtoE)
res = compareVectors(identity, RRt)
assert(res)

position_inertial = [[100],[10],[-10]] # position 100 units from center of earth, up 10, east 10
position = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0])
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, position)

position_of = mm.multiply(R_EtoO, position_inertial)
position_of_exp = [[0],[math.hypot(10,10)],[100]] # expected that the R component is equal to the distance from center
                                   # O is the vertical offset

res = compareVectors(position_of, position_of_exp)
assert(res)

identity = [[1,0,0],[0,1,0],[0,0,1]]
RRt = mm.multiply(R_EtoO, R_OtoE)
res = compareVectors(identity, RRt)
assert(res)

######################################################################################################################################

# testing getOrbitalAxisVals with orbit around equator
orbit = [[0],[0],[-1]] # defines a counter clockwise orbit around the equator
position_inertial = [[100],[0],[0]] # position 100 units from center of earth
vs = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0],
                         u=30, v=40, w=50)
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, vs)

ORB_Pos, ORB_Vel = of.getOrbitalAxisVals(R_EtoO, R_OtoE, vs)

ORB_Pos_exp = [[0],[0],[100]] # expected that the R component is equal to the distance from center
res = compareVectors(ORB_Pos, ORB_Pos_exp)
assert(res)

ORB_Vel_exp = [[-40],[-50],[30]]
res = compareVectors(ORB_Vel, ORB_Vel_exp)
assert(res)

# testing getOrbitalAxisVals with pitched robot
orbit = [[0],[0],[-1]] # defines a counter clockwise orbit around the equator
position_inertial = [[100],[0],[0]] # position 100 units from center of earth
vs = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0],
                         u=30, v=40, w=50, pitch=math.radians(45))
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, vs)

ORB_Pos, ORB_Vel = of.getOrbitalAxisVals(R_EtoO, R_OtoE, vs)

ORB_Pos_exp = [[0],[0],[100]] # expected that the R component is equal to the distance from center
res = compareVectors(ORB_Pos, ORB_Pos_exp)
assert(res)

ORB_Vel_exp = [[-40],[-20*math.sin(math.radians(45))],[80*math.cos(math.radians(45))]]
res = compareVectors(ORB_Vel, ORB_Vel_exp)
assert(res)


# testing getOrbitalAxisVals with pitched orbit
orbit = [[0],[0],[-1]]
# The orbit is pitched by 45 degrees about the ECI y axis
R = mm.transpose(Rotations.euler2DCM(0,math.radians(45),0))
orbit = mm.multiply(R, orbit)

position_inertial = [[100],[0],[-100]] # position 100 units from center of earth
vs = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0],
                         u=30, v=40, w=50)
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, vs)

ORB_Pos, ORB_Vel = of.getOrbitalAxisVals(R_EtoO, R_OtoE, vs)

ORB_Pos_exp = [[0],[0],[math.hypot(100,100)]] # expected that the R component is equal to the distance from center
res = compareVectors(ORB_Pos, ORB_Pos_exp)
assert(res)

ORB_Vel_exp = [[-40],[-80*math.cos(math.radians(45))],[-20*math.sin(math.radians(45))]]
res = compareVectors(ORB_Vel, ORB_Vel_exp)
assert(res)


# testing getOrbitalAxisVals with pitched robot and pitched orbit
orbit = [[-1],[0],[-1]] # defines a counter clockwise orbit around the equator
position_inertial = [[100],[0],[-100]] # position 100 units from center of earth
vs = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0],
                         u=30, v=40, w=50, pitch=math.radians(45))
R_EtoO, R_OtoE = of.orbitalFrameR(orbit, vs)

ORB_Pos, ORB_Vel = of.getOrbitalAxisVals(R_EtoO, R_OtoE, vs)

ORB_Pos_exp = [[0],[0],[math.hypot(100,100)]] # expected that the R component is equal to the distance from center
res = compareVectors(ORB_Pos, ORB_Pos_exp)
assert(res)

ORB_Vel_exp = [[-40],[-50],[30]]
res = compareVectors(ORB_Vel, ORB_Vel_exp)
assert(res)

######################################################################################################################################

# Testing getOrbitalAngularVals with orbit around equator
orbit = [[0],[0],[-1]] # defines a counter clockwise orbit around the equator
position_inertial = [[100],[0],[0]] # position 100 units from center of earth
vs = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0],
                         roll=math.radians(-90), pitch=math.radians(0),yaw=math.radians(-90))

R_EtoO, R_OtoE = of.orbitalFrameR(orbit, vs)

yaw, pitch, roll, yawdot, pitchdot, rolldot = of.getOrbitalAngularVals(R_EtoO, R_OtoE, vs)

eulerVec = [[math.degrees(yaw)],[math.degrees(pitch)],[math.degrees(roll)]]
eulerDotVec = [[yawdot],[pitchdot],[rolldot]]


eulerVec_exp = [[math.radians(0)],[math.radians(0)],[math.radians(0)]]
res = compareVectors(eulerVec, eulerVec_exp)

assert(res)

# # Testing getOrbitalAngularVals with orbit around equator
# orbit = [[0],[0],[-1]] # defines a counter clockwise orbit around the equator
# position_inertial = [[100],[0],[0]] # position 100 units from center of earth
# vs = States.vehicleState(pn=position_inertial[0][0], pe=position_inertial[1][0],pd=position_inertial[2][0],
#                          roll=math.radians(20), pitch=math.radians(30),yaw=math.radians(0))

# R_EtoO = Rotations.euler2DCM(math.radians(10), math.radians(30), math.radians(20))
# R_OtoE = mm.transpose(R_EtoO)

# yaw, pitch, roll, yawdot, pitchdot, rolldot = of.getOrbitalAngularVals(R_EtoO, R_OtoE, vs)

# eulerVec = [[math.degrees(yaw)],[math.degrees(pitch)],[math.degrees(roll)]]
# eulerDotVec = [[yawdot],[pitchdot],[rolldot]]


# eulerVec_exp = [[10],[0],[0]]
# res = compareVectors(eulerVec, eulerVec_exp)
# print(eulerVec)
# print(eulerVec_exp)

# assert(res)