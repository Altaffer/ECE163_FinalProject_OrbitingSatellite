from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import OrbitalFrame as of
from ece163.Containers import States
import math

isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)
def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

orbit = [[0],[0],[-1]] # defines a counter clockwise orbit around the equator

# Testing with no orbital offset
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