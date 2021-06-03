import pymap3d as pm
import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
from ..Modeling import VehicleGravitationalModel as VGM
from  matplotlib  import  pyplot  as plt

def runTest(dT, time, startN, startE, startD, controlSettings, gravityCntrl, controlsCntrl, disturbancesCntrl):
    # SIMULATION PROFILE
    gravModel = VGM.VehicleGravitationalModel(initialNorth=startN, initialEast=startE, initialDown=startD,
                 gravity = gravityCntrl, controls = controlsCntrl, disturbances = disturbancesCntrl)


    # GRAPH VARIOUS STATE VALUES OVER 10 SECONDS
    # define time steps and total time
    dT = dT
    T_tot = time
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

    #location/speed data
    data_speed = [0 for i in range(n_steps)]
    data_lat = [0 for i in range(n_steps)]
    data_long = [0 for i in range(n_steps)]
    data_alt = [0 for i in range(n_steps)]

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
        data_lat[i], data_long[i], data_alt[i] = pm.ned2geodetic(data_pn[i], data_pe[i], data_pd[i], 0, 0, 0)

        # get forces data
        if gravityCntrl:
            data_Fg_x[i] = gravModel.gravityForces(gravModel.getVehicleState()).Fx
            data_Fg_y[i] = gravModel.gravityForces(gravModel.getVehicleState()).Fy
            data_Fg_z[i] = gravModel.gravityForces(gravModel.getVehicleState()).Fz

        if disturbancesCntrl:
            data_Fd_x[i] = gravModel.disturbanceForces(gravModel.getVehicleState()).Fx
            data_Fd_y[i] = gravModel.disturbanceForces(gravModel.getVehicleState()).Fy
            data_Fd_z[i] = gravModel.disturbanceForces(gravModel.getVehicleState()).Fz

        if controlsCntrl:
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
        gravModel.Update(controlSettings)


    # PLOT DATA

    # pn, pe, pd
    fig, points = plt.subplots(3, 1, sharex='all')
    points[0][0].plot(t_data, data_pn)
    points[0][0].set_title("pn")
    points[1][0].plot(t_data, data_pe)
    points[1][0].set_title("pe")
    points[2][0].plot(t_data, data_pd)
    points[2][0].set_title("pd")
    points[2][0].set(xlabel="time (s)")

    # u, v, w
    fig, speeds = plt.subplots(3, 1, sharex='all')
    speeds[0][0].plot(t_data, data_u)
    speeds[0][0].set_title("u")
    speeds[1][0].plot(t_data, data_v)
    speeds[1][0].set_title("v")
    speeds[2][0].plot(t_data, data_w)
    speeds[2][0].set_title("w")
    speeds[2][0].set(xlabel="time (s)")

    # pitch roll yaw
    fig, att = plt.subplots(3, 1, sharex='all')
    att[0][0].plot(t_data, data_pitch)
    att[0][0].set_title("pitch")
    att[1][0].plot(t_data, data_roll)
    att[1][0].set_title("roll")
    att[2][0].plot(t_data, data_yaw)
    att[2][0].set_title("yaw")
    att[2][0].set(xlabel="time (s)")

    # p, q, r
    fig, rates = plt.subplots(3, 1, sharex='all')
    rates[0][0].plot(t_data, data_p)
    rates[0][0].set_title("p")
    rates[1][0].plot(t_data, data_q)
    rates[1][0].set_title("q")
    rates[2][0].plot(t_data, data_r)
    rates[2][0].set_title("r")
    rates[2][0].set(xlabel="time (s)")

    # lat, long, alt
    fig, geo = plt.subplots(3, 1, sharex='all')
    geo[0][0].plot(t_data, data_lat)
    geo[0][0].set_title("lat")
    geo[1][0].plot(t_data, data_long)
    geo[1][0].set_title("long")
    geo[2][0].plot(t_data, data_alt)
    geo[2][0].set_title("alt")
    geo[2][0].set(xlabel="time (s)")

    # speed
    fig, speed = plt.subplots(1, 1, sharex='all')
    speed[0][0].plot(t_data, data_speed)
    speed[0][0].set_title("speed")
    speed[0][0].set(xlabel="time (s)")

    if gravityCntrl:
        # fg
        fig, grav = plt.subplots(3, 1, sharex='all')
        grav[0][0].plot(t_data, data_Fg_x)
        grav[0][0].set_title("Fg x")
        grav[1][0].plot(t_data, data_Fg_y)
        grav[1][0].set_title("Fg y")
        grav[2][0].plot(t_data, data_Fg_z)
        grav[2][0].set_title("Fg z")
        grav[2][0].set(xlabel="time (s)")

    if controlsCntrl:
        # ft
        fig, thrust = plt.subplots(3, 1, sharex='all')
        thrust[0][0].plot(t_data, data_Ft_x)
        thrust[0][0].set_title("Ft x")
        thrust[1][0].plot(t_data, data_Ft_y)
        thrust[1][0].set_title("Ft y")
        thrust[2][0].plot(t_data, data_Ft_z)
        thrust[2][0].set_title("Ft z")
        thrust[2][0].set(xlabel="time (s)")

        # Fr
        fig, reaction = plt.subplots(3, 1, sharex='all')
        reaction[0][0].plot(t_data, data_Fr_x)
        reaction[0][0].set_title("Fr x")
        reaction[1][0].plot(t_data, data_Fr_y)
        reaction[1][0].set_title("Fr y")
        reaction[2][0].plot(t_data, data_Fr_z)
        reaction[2][0].set_title("Fr z")
        reaction[2][0].set(xlabel="time (s)")

    if disturbancesCntrl:
        # Fd
        fig, disturb = plt.subplots(3, 1, sharex='all')
        disturb[0][0].plot(t_data, data_Fd_x)
        disturb[0][0].set_title("Fd x")
        disturb[1][0].plot(t_data, data_Fd_y)
        disturb[1][0].set_title("Fd y")
        disturb[2][0].plot(t_data, data_Fd_z)
        disturb[2][0].set_title("Fd z")
        disturb[2][0].set(xlabel="time (s)")

    plt.show()
    return

