import math
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Modeling import VehicleDynamicsModel
from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import Rotations
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Modeling import VehicleGravitationalModel as VGM
from  matplotlib  import  pyplot  as plt


# TO RUN TESTS/SIMULATIONS, SCROLL TO THE BOTTOM TO INPUT A TEST

class testArgs():
    def __init__(self, dT=50, time=86400, \
        startU=0, startV=0, startW=0, \
        startPn=0, startPe=0, startPd=0, \
        startRoll=0, startPitch=0, startYaw=0,\
        gravityCntrl=0, controlsCntrl=0, disturbancesCntrl=0, \
        controlSettings=Inputs.controlInputs()):
        self.dT = dT # time step between each plotted point in seconds
                     # this is different from the VGM time step
                     # a dT of 1 would have 100 physics steps
                     # between each plotted point if the VGM dT = .01
        self.time = time # duration of the simulation in seconds
        self.startU = startU # starting velocity in the body frame X
        self.startV = startV # starting velocity in the body frame Y
        self.startW = startW # starting velocity in the body frame Z
        self.startPn = startPn # starting position along ECI X axis
        self.startPe = startPe # starting position along ECI Y axis
        self.startPd = startPd # starting position along ECI Z axis
        self.startRoll = startRoll #starting roll from ECI to body
        self.startPitch = startPitch #starting pitch from ECI to body
        self.startYaw = startYaw #starting yaw from ECI to body
        
        self.gravityCntrl = gravityCntrl # set to 1 to include gravity forces in simulation
                                         # set to 0 to ignore gravity forces
        self.controlsCntrl = controlsCntrl # set to 1 to include control input forces
                                           # set to 0 to ignore
        self.disturbancesCntrl = disturbancesCntrl # set to 1 to include disturbances forces
                                                   # set to 0 to ignore
        self.controlSettings = controlSettings # static control inputs
                                               # for example, you could see what happens to the
                                               # sattelite if one of the thrusters remains permanently at
                                               # half power
        return


def runTest(testArgs):
    # SIMULATION PROFILE
    gravModel = VGM.VehicleGravitationalModel(initialNorth=testArgs.startPn, initialEast=testArgs.startPe, initialDown=testArgs.startPd,
                                              initialU=testArgs.startU, initialV=testArgs.startV, initialW=testArgs.startW, 
                                              gravity = testArgs.gravityCntrl, controls = testArgs.controlsCntrl, disturbances = testArgs.disturbancesCntrl)


    # GRAPH VARIOUS STATE VALUES OVER 10 SECONDS
    # define time steps and total time
    dT = testArgs.dT
    T_tot = testArgs.time
    n_steps = int(T_tot / dT)

    # define datasets
    t_data = [i * dT for i in range(n_steps)]

    #state data
    data_pd = [0 for i in range(n_steps)]
    data_pn = [0 for i in range(n_steps)]
    data_pe = [0 for i in range(n_steps)]
    data_u = [0 for i in range(n_steps)]
    data_v = [0 for i in range(n_steps)]
    data_w = [0 for i in range(n_steps)]
    data_pitch = [0 for i in range(n_steps)]
    data_roll = [0 for i in range(n_steps)]
    data_yaw = [0 for i in range(n_steps)]
    data_p = [0 for i in range(n_steps)]
    data_q = [0 for i in range(n_steps)]
    data_r = [0 for i in range(n_steps)]

    #speed/ altitude data
    data_speed = [0 for i in range(n_steps)]
    data_altitude = [0 for i in range(n_steps)]

    #forces data
    data_Fg_x = [0 for i in range(n_steps)]
    data_Fg_y = [0 for i in range(n_steps)]
    data_Fg_z = [0 for i in range(n_steps)]
    data_Ft_x = [0 for i in range(n_steps)]
    data_Ft_y = [0 for i in range(n_steps)]
    data_Ft_z = [0 for i in range(n_steps)]
    data_Fr_x = [0 for i in range(n_steps)]
    data_Fr_y = [0 for i in range(n_steps)]
    data_Fr_z = [0 for i in range(n_steps)]
    data_Fd_x = [0 for i in range(n_steps)]
    data_Fd_y = [0 for i in range(n_steps)]
    data_Fd_z = [0 for i in range(n_steps)]


    # FILL DATASETS
    # update repeatedly over time interval
    for i in range(n_steps):

        # record data - FILL WITH THE SPECIFIC DATA YOU ARE LOOKING FOR,
        # ex: data[i] = gravModel.getVehicleState().pd   for the height

        # get state data
        data_pn[i] = gravModel.getVehicleState().pn
        data_pe[i] = gravModel.getVehicleState().pe
        data_pd[i] = gravModel.getVehicleState().pd

        data_u[i] = gravModel.getVehicleState().u
        data_v[i] = gravModel.getVehicleState().v
        data_w[i] = gravModel.getVehicleState().w

        data_pitch[i] = gravModel.getVehicleState().pitch
        data_roll[i] = gravModel.getVehicleState().roll
        data_yaw[i] = gravModel.getVehicleState().yaw

        data_p[i] = gravModel.getVehicleState().p
        data_q[i] = gravModel.getVehicleState().q
        data_r[i] = gravModel.getVehicleState().r

        # get speed/location data
        data_speed[i] = math.hypot(data_u[i], data_v[i], data_w[i])
        data_altitude[i] = math.hypot(data_pn[i],data_pe[i],data_pd[i]) - VPC.radius_e

        # get forces data
        if testArgs.gravityCntrl:
            data_Fg_x[i] = gravModel.gravityForces(gravModel.getVehicleState()).Fx / VPC.mass
            data_Fg_y[i] = gravModel.gravityForces(gravModel.getVehicleState()).Fy / VPC.mass
            data_Fg_z[i] = gravModel.gravityForces(gravModel.getVehicleState()).Fz / VPC.mass

        if testArgs.disturbancesCntrl:
            data_Fd_x[i] = gravModel.disturbanceForces(gravModel.getVehicleState()).Fx
            data_Fd_y[i] = gravModel.disturbanceForces(gravModel.getVehicleState()).Fy
            data_Fd_z[i] = gravModel.disturbanceForces(gravModel.getVehicleState()).Fz

        if testArgs.controlsCntrl:
            controlSettings = testArgs.controlSettings
            data_Ft_x[i] = gravModel.calculateThrustersForces(controlSettings.ThrusterX, controlSettings.ThrusterY,
                                                              controlSettings.ThrusterZ).Fx
            data_Ft_y[i] = gravModel.calculateThrustersForces(controlSettings.ThrusterX, controlSettings.ThrusterY,
                                                              controlSettings.ThrusterZ).Fy
            data_Ft_z[i] = gravModel.calculateThrustersForces(controlSettings.ThrusterX, controlSettings.ThrusterY,
                                                              controlSettings.ThrusterZ).Fz

            data_Fr_x[i] = gravModel.calculateReactionWheelForces(controlSettings.ReactionX, controlSettings.ReactionY,
                                                                  controlSettings.ReactionZ).Mx
            data_Fr_y[i] = gravModel.calculateReactionWheelForces(controlSettings.ReactionX, controlSettings.ReactionY,
                                                                  controlSettings.ReactionZ).My
            data_Fr_z[i] = gravModel.calculateReactionWheelForces(controlSettings.ReactionX, controlSettings.ReactionY,
                                                                  controlSettings.ReactionZ).Mz


        # update the gravitational model
        gravModel.Update(testArgs.controlSettings)


    # PLOT DATA

    # pn, pe, pd
    fig, points = plt.subplots(3, 1, sharex='all')
    points[0].plot(t_data, data_pn)
    points[0].set_title("pn")
    points[1].plot(t_data, data_pe)
    points[1].set_title("pe")
    points[2].plot(t_data, data_pd)
    points[2].set_title("pd")
    points[2].set(xlabel="time (s)")

    # u, v, w
    fig, speeds = plt.subplots(3, 1, sharex='all')
    speeds[0].plot(t_data, data_u)
    speeds[0].set_title("u")
    speeds[1].plot(t_data, data_v)
    speeds[1].set_title("v")
    speeds[2].plot(t_data, data_w)
    speeds[2].set_title("w")
    speeds[2].set(xlabel="time (s)")

    # pitch roll yaw
    fig, att = plt.subplots(3, 1, sharex='all')
    att[0].plot(t_data, data_pitch)
    att[0].set_title("pitch")
    att[1].plot(t_data, data_roll)
    att[1].set_title("roll")
    att[2].plot(t_data, data_yaw)
    att[2].set_title("yaw")
    att[2].set(xlabel="time (s)")

    # p, q, r
    fig, rates = plt.subplots(3, 1, sharex='all')
    rates[0].plot(t_data, data_p)
    rates[0].set_title("p")
    rates[1].plot(t_data, data_q)
    rates[1].set_title("q")
    rates[2].plot(t_data, data_r)
    rates[2].set_title("r")
    rates[2].set(xlabel="time (s)")

    # speed and alt
    fig, speed = plt.subplots(2, 1, sharex='all')
    speed[0].plot(t_data, data_speed)
    speed[0].set_title("speed")
    speed[1].plot(t_data, data_altitude)
    speed[1].set_title("altitude")
    speed[1].set(xlabel="time (s)")

    if testArgs.gravityCntrl:
        # fg
        fig, grav = plt.subplots(3, 1, sharex='all')
        grav[0].plot(t_data, data_Fg_x)
        grav[0].set_title("grav acc x")
        grav[1].plot(t_data, data_Fg_y)
        grav[1].set_title("grav acc y")
        grav[2].plot(t_data, data_Fg_z)
        grav[2].set_title("grav acc z")
        grav[2].set(xlabel="time (s)")

    if testArgs.controlsCntrl:
        # ft
        fig, thrust = plt.subplots(3, 1, sharex='all')
        thrust[0].plot(t_data, data_Ft_x)
        thrust[0].set_title("Ft x")
        thrust[1].plot(t_data, data_Ft_y)
        thrust[1].set_title("Ft y")
        thrust[2].plot(t_data, data_Ft_z)
        thrust[2].set_title("Ft z")
        thrust[2].set(xlabel="time (s)")

        # Fr
        fig, reaction = plt.subplots(3, 1, sharex='all')
        reaction[0].plot(t_data, data_Fr_x)
        reaction[0].set_title("Fr x")
        reaction[1].plot(t_data, data_Fr_y)
        reaction[1].set_title("Fr y")
        reaction[2].plot(t_data, data_Fr_z)
        reaction[2].set_title("Fr z")
        reaction[2].set(xlabel="time (s)")

    if testArgs.disturbancesCntrl:
        # Fd
        fig, disturb = plt.subplots(3, 1, sharex='all')
        disturb[0].plot(t_data, data_Fd_x)
        disturb[0].set_title("Fd x")
        disturb[1].plot(t_data, data_Fd_y)
        disturb[1].set_title("Fd y")
        disturb[2].plot(t_data, data_Fd_z)
        disturb[2].set_title("Fd z")
        disturb[2].set(xlabel="time (s)")

    plt.show()
    return

# A given test can be run by constructing a testArgs class and passing it into the runTest function
# For a description of what each parameter does, look to the top of the file to view the testArgs class
# parameter descriptions
# An example test would be placing the satellite at an orbit of 400km, turning off the controls and disturbances
# and observing as gravity causes it to plummet towards the earth

args = testArgs()
args.dT = 50
args.time = 86400
args.startPn = 00e3 + VPC.radius_e
args.gravityCntrl = 1
runTest(args)
