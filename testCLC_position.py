from ece163 import Containers
from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import Rotations
from ece163.Utilities import OrbitalFrame as of
from ece163.Containers import Inputs, States
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Modeling import VehicleGravitationalModel as VGM
from ece163.Constants import VehiclePhysicalConstants as VPC
import math
from  matplotlib  import  pyplot  as plt


gravModel = VGM.VehicleGravitationalModel(gravity=False, disturbances=False,controls=True,
                                          initialNorth=0, initialEast=1000, initialDown=6000000,
                                          initialU=1000, initialV=0, initialW=0)


controlModel = VCLC.VehicleClosedLoopControl()
cg = VCLC.ControlGains()
controlModel.setControlGains(cg)

dT = .01
# T_tot = 800000 # 200 hours
T_tot = 80 # 200 hours
n_steps = int(T_tot / dT)
t_data = [i * dT for i in range(n_steps)]

data_pt_tor = [0 for i in range(n_steps)]
data_po_tor = [0 for i in range(n_steps)]
data_pr_tor = [0 for i in range(n_steps)]
data_velt_tor = [0 for i in range(n_steps)]
data_velo_tor = [0 for i in range(n_steps)]
data_velr_tor = [0 for i in range(n_steps)]

data_pt_tor_com = [0 for i in range(n_steps)]
data_po_tor_com = [0 for i in range(n_steps)]
data_pr_tor_com = [0 for i in range(n_steps)]
data_velt_tor_com = [0 for i in range(n_steps)]
data_velo_tor_com = [0 for i in range(n_steps)]
data_velr_tor_com = [0 for i in range(n_steps)]

for i in range(n_steps):
    vs = gravModel.getVehicleState()
    # we're assuming that the orientation controls are working properly so the body is aligned with the orbital frame
    R_e2o = [[1,0,0],[0,1,0],[0,0,1]]
    R_o2e = R_e2o
    thrusterXcontrol, thrusterYcontrol, thrusterZcontrol = controlModel.controlPosition(vs, R_e2o, R_o2e)
    controls = Inputs.controlInputs(ThrusterX=thrusterXcontrol, ThrusterY=thrusterYcontrol, ThrusterZ=thrusterZcontrol)

    gravModel.Update(controls)

    orbPos, orbVel = of.getOrbitalAxisVals(R_e2o, R_o2e, vs)

    data_pr_tor_com[i] = math.hypot(controlModel.OrbitVector[0][0], controlModel.OrbitVector[1][0], controlModel.OrbitVector[2][0])

    data_pt_tor[i], data_po_tor[i], data_pr_tor[i] = mm.transpose(orbPos)[0]
    data_velt_tor[i], data_velo_tor[i], data_velr_tor[i] = mm.transpose(orbVel)[0]

fig, positionTOR = plt.subplots(3, 1, sharex='all')
positionTOR[0].plot(t_data, data_pt_tor)
positionTOR[0].set_title("orbital position pt")
positionTOR[1].plot(t_data, data_po_tor, label='current')
positionTOR[1].plot(t_data, data_po_tor_com, label='commanded')
positionTOR[1].legend()
positionTOR[1].set_title("orbital position po")
positionTOR[2].plot(t_data, data_pr_tor, label='current')
positionTOR[2].plot(t_data, data_pr_tor_com, label='commanded')
positionTOR[2].legend()
positionTOR[2].set_title("orbital position pr")
positionTOR[2].set(xlabel="time (s)")

fig, velTOR = plt.subplots(3, 1, sharex='all')
velTOR[0].plot(t_data, data_velt_tor, label='current')
velTOR[0].plot(t_data, data_velt_tor_com, label='commanded')
velTOR[0].legend()
velTOR[0].set_title("orbital vel pt")
velTOR[1].plot(t_data, data_velo_tor)
velTOR[1].set_title("orbital vel po")
velTOR[2].plot(t_data, data_velr_tor)
velTOR[2].set_title("orbital vel pr")
velTOR[2].set(xlabel="time (s)")

# fig, circle = plt.subplots()
# circle.plot(data_pn, data_pe)
# positionE[0].set_title("eci pe vs pd")

plt.show()
