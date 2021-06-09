import math
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Modeling import VehicleDynamicsModel
from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import Rotations
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Modeling import MoonGravitationalModel as MGM
from ece163.Modeling import DisturbancesModel as DM
from  matplotlib  import  pyplot  as plt


# TO RUN TESTS/SIMULATIONS, SCROLL TO THE BOTTOM TO INPUT A TEST

class testArgs():
    def __init__(self, dT=50, time=86400, \
                 startU=0, startV=0, startW=0, \
                 startPn=0, startPe=0, startPd=0, \
                 startRoll=0, startPitch=0, startYaw=0,\
                 gravityCntrl=0):
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
        return


def runTest(args:testArgs):
    # SIMULATION PROFILE
    gravModel = MGM.MoonGravitationalModel(initialNorth=args.startPn, initialEast=args.startPe, initialDown=args.startPd,
                                              initialU=args.startU, initialV=args.startV, initialW=args.startW, 
                                              gravity = args.gravityCntrl)

    gravModel.getMoonState().yaw = args.startYaw
    gravModel.getMoonState().pitch = args.startPitch
    gravModel.getMoonState().roll = args.startRoll

    gravModel.getMoonDynamicsModel().dT = args.dT

    gravModel.getMoonState().R = Rotations.euler2DCM(args.startYaw, args.startPitch, args.startRoll)
    # GRAPH VARIOUS STATE VALUES OVER 10 SECONDS
    # define time steps and total time
    dT = args.dT
    T_tot = args.time
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
        # ex: data[i] = gravModel.getMoonState().pd   for the height

        # get state data
        data_pn[i] = gravModel.getMoonState().pn
        data_pe[i] = gravModel.getMoonState().pe
        data_pd[i] = gravModel.getMoonState().pd

        data_u[i] = gravModel.getMoonState().u
        data_v[i] = gravModel.getMoonState().v
        data_w[i] = gravModel.getMoonState().w

        data_pitch[i] = gravModel.getMoonState().pitch
        data_roll[i] = gravModel.getMoonState().roll
        data_yaw[i] = gravModel.getMoonState().yaw

        data_p[i] = gravModel.getMoonState().p
        data_q[i] = gravModel.getMoonState().q
        data_r[i] = gravModel.getMoonState().r

        # get speed/location data
        data_speed[i] = math.hypot(data_u[i], data_v[i], data_w[i])
        data_altitude[i] = math.hypot(data_pn[i],data_pe[i],data_pd[i]) - VPC.radius_e

        # get forces data
        if args.gravityCntrl:
            data_Fg_x[i] = gravModel.gravityForces(gravModel.getMoonState()).Fx / VPC.mass
            data_Fg_y[i] = gravModel.gravityForces(gravModel.getMoonState()).Fy / VPC.mass
            data_Fg_z[i] = gravModel.gravityForces(gravModel.getMoonState()).Fz / VPC.mass


        # update the gravitational model
        gravModel.Update()


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

    if args.gravityCntrl:
        # fg
        fig, grav = plt.subplots(3, 1, sharex='all')
        grav[0].plot(t_data, data_Fg_x)
        grav[0].set_title("grav acc x")
        grav[1].plot(t_data, data_Fg_y)
        grav[1].set_title("grav acc y")
        grav[2].plot(t_data, data_Fg_z)
        grav[2].set_title("grav acc z")
        grav[2].set(xlabel="time (s)")

    plt.show()

    #animation for NE plane
    x_data = []
    y_data = []

    earth = plt.Circle((0, 0), VPC.radius_e / 1e6, color='green', linewidth=1)

    days = 0
    hours = 0
    mins = 0
    secs = 0
    simsecs = 0
    simtime = 0

    for i in range(n_steps):
        #collect data
        x_data.append(data_pe[i] / 1e6)
        y_data.append(data_pn[i] / 1e6)

        # calculate days elapsed
        if hours == 24:
            days += 1
            hours = 0
        # calculate hours elapsed
        if secs >= 3600:
            hours += 1
            secs -= 3600
        secs += args.dT
        simtime += 1

        if i % 20 == 0:
            #plot earth, set graph limits, set title, plot data
            plt.gca().add_patch(earth)
            plt.xlim(-600, 600)
            plt.ylim(-600, 600)
            plt.plot(x_data, y_data, color='blue', linewidth=1)
            plt.title("Days elapsed: {days}   Hours elapsed: {hours} ,    SimTime = {simtime} steps" \
                         .format(days=days, hours=hours, simtime=simtime))
            plt.xlabel("Position east (million meters)")
            plt.ylabel("Position north (million meters)")
            plt.pause(0.001)
    plt.show()
    return

# A given test can be run by constructing a testArgs class and passing it into the runTest function
# For a description of what each parameter does, look to the top of the file to view the testArgs class
# parameter descriptions
# An example test would be placing the moon at an orbit of 4 million km
# and observing as gravity causes it to plummet towards the earth

args = testArgs()
args.dT = 50
args.time = 86400
args.startPe = 384e6 + VPC.radius_e
args.startU = 1022
args.gravityCntrl = 1
runTest(args)


# Testing MoonGravitationalModel affect on DisturbancesModel
def DMtests():
    # tests reagrding the position of the moon in the disturbance model
    MGM.MoonGravitationalModel(initialNorth=VPC.MoonInitialNorth, initialEast=VPC.MoonInitialEast,
                 initialDown=VPC.MoonInitialDown, initialU=VPC.MoonInitialU, initialV=VPC.MoonInitialV,
                 initialW=VPC.MoonInitialW, gravity = True)
    state = States.vehicleState()
    state.pd = 4e6
    state.pn = 0
    state.pe = 0
    distance = DM.distanceFromMoon(state)
    print(distance)
DMtests()
