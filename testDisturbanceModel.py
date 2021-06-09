import math
from ece163.Modeling import DisturbancesModel as DM
from ece163.Containers import States
from ece163.Constants import VehiclePhysicalConstants as VPC


isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))


failed = []
passed = []
def evaluateTest(test_name, boolean):
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean


#Test time

# Testing DisturbanceModel.airdrag()
print(" Beginning testing of DisturbanceModel.airdrag()")
cur_test_airdrag = " airdrag() test 1"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 400000.00 # [m]
ad = DM.airdrag(k)
F_drag = 9.101231930648247e-11
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 2"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 800000.00 # [m]
ad = DM.airdrag(k)
F_drag = 3.866525740096517e-28
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 3"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 3000.00 # [m]
ad = DM.airdrag(k)
F_drag = 15870518.416576123
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 4"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 700.00 # [m]
ad = DM.airdrag(k)
F_drag = 19974634.63668867
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 5"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 20.00 # [m]
ad = DM.airdrag(k)
F_drag = 21380155.96465008
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 6"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 90000.00 # [m]
ad = DM.airdrag(k)
F_drag = 2643.803181219505
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 7"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 1000000.00 # [m]
ad = DM.airdrag(k)
F_drag = 7.969503535451863e-37
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 8"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 303030.00 # [m]
ad = DM.airdrag(k)
F_drag = 1.480654643080455e-06
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 9"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 100000.00 # [m]
ad = DM.airdrag(k)
F_drag = 972.600836874313
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")

cur_test_airdrag = " airdrag() test 10"
k = States.vehicleState()
k.Va = 7777 # m/s
k.pd = 280000.00 # [m]
ad = DM.airdrag(k)
F_drag = 1.4812691045286339e-05
if not evaluateTest( cur_test_airdrag , ad == F_drag):
    print(f"{ad} != {F_drag}")









