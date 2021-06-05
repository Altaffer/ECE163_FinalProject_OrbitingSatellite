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
gravModel = VGM.VehicleGravitationalModel(gravity=False, disturbances=False)
gravModel.setVehicleState(vs)
# gravModel.getVehicleState().q = .07

controlModel = VCLC.VehicleClosedLoopControl()
cg = VCLC.ControlGains()
cg.Roll_kp = 6
cg.Roll_kd = 25
cg.Roll_ki = .001

cg.Pitch_kp = 6
cg.Pitch_kd = 25
cg.Pitch_ki = .001

cg.Yaw_kp = 6
cg.Yaw_kd = 25
cg.Yaw_ki = .001
controlModel.setControlGains(cg)

dT = .01
T_tot = 80
n_steps = int(T_tot / dT)

# testing control angular
orbitalYaw = [20 if (i>100) else 0 for i in range(n_steps)]
orbitalPitch = [20 if (i>100) else 0 for i in range(n_steps)]
orbitalRoll = [20 if (i>100) else 0 for i in range(n_steps)]

# define datasets
t_data = [i * dT for i in range(n_steps)]

data_yaw = [0 for i in range(n_steps)]
data_pitch = [0 for i in range(n_steps)]
data_roll = [0 for i in range(n_steps)]

data_xMisalign = [0 for i in range(n_steps)]
data_yMisalign = [0 for i in range(n_steps)]
data_zMisalign = [0 for i in range(n_steps)]

data_xMisalign_dot = [0 for i in range(n_steps)]
data_yMisalign_dot = [0 for i in range(n_steps)]
data_zMisalign_dot = [0 for i in range(n_steps)]

data_xMisalign_dot_controller_observed = [0 for i in range(n_steps)]
data_yMisalign_dot_controller_observed = [0 for i in range(n_steps)]
data_zMisalign_dot_controller_observed = [0 for i in range(n_steps)]

data_xMisalign_exp = [0 for i in range(n_steps)]
data_yMisalign_exp = [0 for i in range(n_steps)]
data_zMisalign_exp = [0 for i in range(n_steps)]

data_reactorXcontrol = [0 for i in range(n_steps)]
data_reactorYcontrol = [0 for i in range(n_steps)]
data_reactorZcontrol = [0 for i in range(n_steps)]

data_p = [0 for i in range(n_steps)]
data_q = [0 for i in range(n_steps)]
data_r = [0 for i in range(n_steps)]

for i in range(n_steps):
    Re2o = Rotations.euler2DCM(math.radians(orbitalYaw[i]), math.radians(orbitalPitch[i]), math.radians(orbitalRoll[i]))
    Ro2e = mm.transpose(Re2o)
    reactorXcontrol, reactorYcontrol, reactorZcontrol = controlModel.controlOrientation(gravModel.getVehicleState(), Re2o, Ro2e)
    controls = Inputs.controlInputs(ThrusterX=0, ReactionX=reactorXcontrol, ReactionY=reactorYcontrol, ReactionZ=reactorZcontrol)
    gravModel.Update(controls)


    data_yaw[i] = math.degrees(gravModel.getVehicleState().yaw)
    data_pitch[i] = math.degrees(gravModel.getVehicleState().pitch)
    data_roll[i] = math.degrees(gravModel.getVehicleState().roll)

    zMisalign, yMisalign, xMisalign, zMisalignDot, yMisalignDot, xMisalignDot = \
        of.getOrbitalAngularVals(Re2o, Ro2e,gravModel.getVehicleState())

    data_xMisalign[i] = math.degrees(xMisalign)
    data_yMisalign[i] = math.degrees(yMisalign)
    data_zMisalign[i] = math.degrees(zMisalign)

    data_xMisalign_dot[i] = math.degrees(xMisalignDot)
    data_yMisalign_dot[i] = math.degrees(yMisalignDot)
    data_zMisalign_dot[i] = math.degrees(zMisalignDot)

    data_xMisalign_exp[i] = orbitalRoll[i] - data_roll[i]
    data_yMisalign_exp[i] = orbitalPitch[i] - data_pitch[i]
    data_zMisalign_exp[i] = orbitalYaw[i] - data_yaw[i]

    data_reactorXcontrol[i] = reactorXcontrol
    data_reactorYcontrol[i] = reactorYcontrol
    data_reactorZcontrol[i] = reactorZcontrol

    data_xMisalign_dot[i] = math.degrees(controlModel.reactorXFromRoll.dot_prev)
    data_yMisalign_dot[i] = math.degrees(controlModel.reactorYFromPitch.dot_prev)
    data_zMisalign_dot[i] = math.degrees(controlModel.reactorZFromYaw.dot_prev)


fig, angularMisalign = plt.subplots(3, 1, sharex='all')
angularMisalign[0].plot(t_data, data_zMisalign,label="actual")
# angularMisalign[0].plot(t_data, data_zMisalign_exp,label="expected")
angularMisalign[0].legend()
angularMisalign[0].set_title("z misalign")
angularMisalign[1].plot(t_data, data_yMisalign,label="actual")
# angularMisalign[1].plot(t_data, data_yMisalign_exp,label="expected")
angularMisalign[1].legend()
angularMisalign[1].set_title("y misalign")
angularMisalign[2].plot(t_data, data_xMisalign,label="actual")
# angularMisalign[2].plot(t_data, data_xMisalign_exp,label="expected")
angularMisalign[2].set_title("x misalign")
angularMisalign[2].legend()
angularMisalign[2].set(xlabel="time (s)")

fig, angularMisalign_dot = plt.subplots(3, 1, sharex='all')
angularMisalign_dot[0].plot(t_data, data_zMisalign_dot,label="actual")
# angularMisalign[0].plot(t_data, data_zMisalign_exp,label="expected")
angularMisalign_dot[0].legend()
angularMisalign_dot[0].set_title("z misalign_dot")
angularMisalign_dot[1].plot(t_data, data_yMisalign_dot,label="actual")
# angularMisalign[1].plot(t_data, data_yMisalign_exp,label="expected")
angularMisalign_dot[1].legend()
angularMisalign_dot[1].set_title("y misalign_dot")
angularMisalign_dot[2].plot(t_data, data_xMisalign_dot,label="actual")
# angularMisalign[2].plot(t_data, data_xMisalign_exp,label="expected")
angularMisalign_dot[2].set_title("x misalign_dot")
angularMisalign_dot[2].legend()
angularMisalign_dot[2].set(xlabel="time (s)")



fig, angularMisalign_dot_controller_obsrved = plt.subplots(3, 1, sharex='all')
angularMisalign_dot_controller_obsrved[0].plot(t_data, data_zMisalign_dot_controller_observed,label="actual")
# angularMisalign[0].plot(t_data, data_zMisalign_exp,label="expected")
angularMisalign_dot_controller_obsrved[0].legend()
angularMisalign_dot_controller_obsrved[0].set_title("z misalign_dot_controller_observed")
angularMisalign_dot_controller_obsrved[1].plot(t_data, data_yMisalign_dot_controller_observed,label="actual")
# angularMisalign[1].plot(t_data, data_yMisalign_exp,label="expected")
angularMisalign_dot_controller_obsrved[1].legend()
angularMisalign_dot_controller_obsrved[1].set_title("y misalign_dot_controller_observed")
angularMisalign_dot_controller_obsrved[2].plot(t_data, data_xMisalign_dot_controller_observed,label="actual")
# angularMisalign[2].plot(t_data, data_xMisalign_exp,label="expected")
angularMisalign_dot_controller_obsrved[2].set_title("x misalign_dot_controller_observed")
angularMisalign_dot_controller_obsrved[2].legend()
angularMisalign_dot_controller_obsrved[2].set(xlabel="time (s)")


fig, angularMisalign = plt.subplots(3, 1, sharex='all')
angularMisalign[0].plot(t_data, data_reactorZcontrol)
angularMisalign[0].set_title("z axis reactor")
angularMisalign[1].plot(t_data, data_reactorYcontrol)
angularMisalign[1].set_title("y axis reactor")
angularMisalign[2].plot(t_data, data_reactorXcontrol)
angularMisalign[2].set_title("x axis reactor")
angularMisalign[2].set(xlabel="time (s)")

fig, eulerAngles = plt.subplots(3, 1, sharex='all')
eulerAngles[0].plot(t_data, data_yaw,label="sat yaw")
eulerAngles[0].plot(t_data, orbitalYaw,label="orb yaw")
eulerAngles[0].set_title("yaw")
eulerAngles[0].legend()
eulerAngles[1].plot(t_data, data_pitch,label="sat pitch")
eulerAngles[1].plot(t_data, orbitalPitch,label="orb pitch")
eulerAngles[1].set_title("pitch")
eulerAngles[1].legend()
eulerAngles[2].plot(t_data, data_roll,label="sat roll")
eulerAngles[2].plot(t_data, orbitalRoll,label="orb roll")
eulerAngles[2].set_title("roll")
eulerAngles[2].legend()
eulerAngles[2].set(xlabel="time (s)")

plt.show()
