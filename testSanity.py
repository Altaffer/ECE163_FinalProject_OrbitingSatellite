from ece163 import Containers
from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import Rotations
from ece163.Utilities import OrbitalFrame as of
from ece163.Containers import Inputs, States
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Modeling import VehicleGravitationalModel as VGM
import math
from  matplotlib  import  pyplot  as plt

satYaw = 0
satPitch = 0
satRoll = 0

vs = States.vehicleState(yaw=math.radians(satYaw),pitch=math.radians(satPitch),roll=math.radians(satRoll))
gravModel = VGM.VehicleGravitationalModel(gravity=False, disturbances=False,controls=False)
gravModel.setVehicleState(vs)
gravModel.getVehicleState().r = .07

dT = .01
T_tot = 40
n_steps = int(T_tot / dT)

t_data = [i * dT for i in range(n_steps)]

data_yaw = [0 for i in range(n_steps)]
data_pitch = [0 for i in range(n_steps)]
data_roll = [0 for i in range(n_steps)]

data_p = [0 for i in range(n_steps)]
data_q = [0 for i in range(n_steps)]
data_r = [0 for i in range(n_steps)]

for i in range(n_steps):
    data_yaw[i] = math.degrees(gravModel.getVehicleState().yaw)
    data_pitch[i] = math.degrees(gravModel.getVehicleState().pitch)
    data_roll[i] = math.degrees(gravModel.getVehicleState().roll)

    data_p[i] = math.degrees(gravModel.getVehicleState().p)
    data_q[i] = math.degrees(gravModel.getVehicleState().q)
    data_r[i] = math.degrees(gravModel.getVehicleState().r)

    controls = Inputs.controlInputs

    gravModel.Update(controls)

fig, eulerAngles = plt.subplots(3, 1, sharex='all')
eulerAngles[0].plot(t_data, data_yaw,label="sat yaw")
eulerAngles[0].set_title("yaw")
eulerAngles[1].plot(t_data, data_pitch,label="sat pitch")
eulerAngles[1].set_title("pitch")
eulerAngles[2].plot(t_data, data_roll,label="sat roll")
eulerAngles[2].set_title("roll")
eulerAngles[2].set(xlabel="time (s)")

fig, eulerRates = plt.subplots(3, 1, sharex='all')
eulerRates[0].plot(t_data, data_r,label="sat r")
eulerRates[0].set_title("sat r")
eulerRates[1].plot(t_data, data_q,label="sat q")
eulerRates[1].set_title("sat q")
eulerRates[2].plot(t_data, data_p,label="sat p")
eulerRates[2].set_title("sat p")
eulerRates[2].set(xlabel="time (s)")

plt.show()
