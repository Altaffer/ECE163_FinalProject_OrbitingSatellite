from ece163.Utilities import MatrixMath as mm
import math

isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)


a = [[1],[2],[3]]
b = [[4],[5],[6]]

c = mm.vectorProjection(a,b)
c_exp = [[128/77], [160/77], [192/77]]

res = compareVectors(c, c_exp)
assert(res)

d = mm.vectorRejection(a, b)
d_exp = mm.subtract(a, c_exp)

res = compareVectors(d, d_exp)
assert(res)