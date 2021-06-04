from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import OrbitalFrame as of
from ece163.Containers import States
from ece163.Controls import VehicleClosedLoopControl as VCLC
import math

controller = VCLC.VehicleClosedLoopControl()
controller.setControlGains()

vs = States.vehicleState(pn=1,pe=2,pd=3,u=1)

controller.UpdateControlCommands(vehicleState=vs)