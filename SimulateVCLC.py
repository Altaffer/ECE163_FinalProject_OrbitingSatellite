import math
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Modeling import VehicleDynamicsModel
from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import Rotations
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Modeling import VehicleGravitationalModel as VGM
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Utilities import OrbitalFrame as OF
from matplotlib import pyplot  as plt


# TO RUN TESTS/SIMULATIONS, SCROLL TO THE BOTTOM TO INPUT A TEST

class testArgs():
    def __init__(self, dT=50, time=86400, startOrbitalSpeed=7000, orbitVector=[[0],[0],[-(400000+VPC.radius_e)]], \
                 orbitStartPosNED=[[0], [400000+VPC.radius_e], [0]],  controlGains = VCLC.ControlGains(), \
                 gravityCntrl=0, controlsCntrl=0, disturbancesCntrl=0, returnECIdata=0, returnTORdata=1):
        self.dT = dT  # time step between each plotted point in seconds
        # this is different from the VGM time step
        # a dT of 1 would have 100 physics steps
        # between each plotted point if the VGM dT = .01
        self.time = time  # duration of the simulation in seconds
        self.startOrbitalSpeed = startOrbitalSpeed  # the starting on-orbit speed the spacecraft has
        self.orbitVector = orbitVector  # the vector tangent to the plane of orbit, this controls the orbit
        self.orbitStartPosNED = orbitStartPosNED  # the starting location of the craft, in NED
        # ** START POSITION NED SHOULD BE ON THE ORBITAL FRAME, i.e perpendicular to orbitVector **
        self.gravityCntrl = gravityCntrl  # set to 1 to include gravity forces in simulation
        # set to 0 to ignore gravity forces
        self.controlsCntrl = controlsCntrl  # set to 1 to include control input forces
        # set to 0 to ignore
        self.disturbancesCntrl = disturbancesCntrl  # set to 1 to include disturbances forces
        # set to 0 to ignore
        self.returnECIdata = returnECIdata  # set to 1 return plots of position, uvw, ypr, pqr in ECI coords
        # set to 0 to ignore
        self.returnTORdata = returnTORdata  # set to 1 to return plots of position, uvw, ypr, pqr, in TOR coords
        # set to 0 to ignore

        #to find the initial speed of the craft in uvw to initialize the VGM, rotate to orbital frame speed to body
        state = States.vehicleState(pn=self.orbitStartPosNED[0][0], pe=self.orbitStartPosNED[1][0],
                                    pd=self.orbitStartPosNED[2][0])  # create a temporary state with NED to get rots
        Reci2orbital, Rorbital2eci = OF.orbitalFrameR(self.orbitVector, state)  # get rots for eci and orbit
        Rbody2orbital, Rorbital2body = OF.getBodyOrbitalRots(Reci2orbital, Rorbital2eci, state)  # get body rotations
        self.speedUVW = mm.multiply(Rorbital2body, [[startOrbitalSpeed], [0], [0]])  # get the uvw speed from orbit

        self.controlGains = controlGains

        return


def runTest(args: testArgs):
    # SIMULATION PROFILE
    gravModel = VGM.VehicleGravitationalModel(initialNorth=args.orbitStartPosNED[0][0],
                                              initialEast=args.orbitStartPosNED[1][0],
                                              initialDown=args.orbitStartPosNED[2][0],
                                              initialU=args.speedUVW[0][0],
                                              initialV=args.speedUVW[1][0],
                                              initialW=args.speedUVW[2][0],
                                              gravity=args.gravityCntrl, controls=args.controlsCntrl,
                                              disturbances=args.disturbancesCntrl)

    clControl = VCLC.VehicleClosedLoopControl(dT=args.dT, OrbitVector=args.orbitVector)
    clControl.setControlGains(args.controlGains)

    # GRAPH VARIOUS STATE VALUES OVER time SECONDS
    # define time steps and total time
    dT = args.dT
    T_tot = args.time
    n_steps = int(T_tot / dT)

    # define datasets
    t_data = [i * dT for i in range(n_steps)]

    # state data
    if args.returnECIdata:
        data_pn = [0 for i in range(n_steps)]
        data_pe = [0 for i in range(n_steps)]
        data_pd = [0 for i in range(n_steps)]
        data_u = [0 for i in range(n_steps)]
        data_v = [0 for i in range(n_steps)]
        data_w = [0 for i in range(n_steps)]
        data_pitch = [0 for i in range(n_steps)]
        data_roll = [0 for i in range(n_steps)]
        data_yaw = [0 for i in range(n_steps)]
        data_p = [0 for i in range(n_steps)]
        data_q = [0 for i in range(n_steps)]
        data_r = [0 for i in range(n_steps)]
    if args.returnTORdata:
        data_pt_tor = [0 for i in range(n_steps)]
        data_po_tor = [0 for i in range(n_steps)]
        data_pr_tor = [0 for i in range(n_steps)]
        data_velt_tor = [0 for i in range(n_steps)]
        data_velo_tor = [0 for i in range(n_steps)]
        data_velr_tor = [0 for i in range(n_steps)]
        data_pitch_tor = [0 for i in range(n_steps)]
        data_roll_tor = [0 for i in range(n_steps)]
        data_yaw_tor = [0 for i in range(n_steps)]
        data_yawdot_tor = [0 for i in range(n_steps)]
        data_pitchdot_tor = [0 for i in range(n_steps)]
        data_rolldot_tor = [0 for i in range(n_steps)]

    # speed/ altitude data
    data_speed = [0 for i in range(n_steps)]
    data_altitude = [0 for i in range(n_steps)]
    data_set_alt = [math.hypot(args.orbitVector[0][0], args.orbitVector[1][0], args.orbitVector[2][0]) for i in range(n_steps)]

    # FILL DATASETS
    # update repeatedly over time interval
    for i in range(n_steps):

        # record data - FILL WITH THE SPECIFIC DATA YOU ARE LOOKING FOR,
        # ex: data[i] = gravModel.getVehicleState().pd   for the height

        # get state data
        if args.returnECIdata:
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

        # to get data in TOR, we need to rotate the above values
        if args.returnTORdata:
            Re2o, Ro2e = OF.orbitalFrameR(args.orbitVector, gravModel.getVehicleState())  # get rotation vectors
            Orb_pos, Orb_vel = OF.getOrbitalAxisVals(Re2o, Ro2e, gravModel.getVehicleState()) # get position & vel in ORB

            data_pt_tor[i] = Orb_pos[0][0]
            data_po_tor[i] = Orb_pos[1][0]
            data_pr_tor[i] = Orb_pos[2][0]

            data_velt_tor[i] = Orb_vel[0][0]
            data_velo_tor[i] = Orb_vel[1][0]
            data_velr_tor[i] = Orb_vel[2][0]

            data_yaw_tor[i], data_pitch_tor[i], data_roll_tor[i], data_yawdot_tor[i], data_pitchdot_tor[i], \
            data_rolldot_tor[i] = OF.getBodyOrbitalRots(Re2o, Ro2e, gravModel.getVehicleState())

        # get speed/location data
        data_speed[i] = math.hypot(gravModel.getVehicleState().u, gravModel.getVehicleState().v,
                                   gravModel.getVehicleState().w)
        data_altitude[i] = math.hypot(gravModel.getVehicleState().pn, gravModel.getVehicleState().pe,
                                      gravModel.getVehicleState().pd) - VPC.radius_e

        #update the controls model
        controls = clControl.UpdateControlCommands(gravModel.getVehicleState())
        # update the gravitational model
        gravModel.Update(controls)

    # PLOT DATA
    if args.returnECIdata and args.returnTORdata:
        # position
        fig, points = plt.subplots(3, 2, sharex='all')
        points[0][0].plot(t_data, data_pn)
        points[0][0].set_title("eci pn")
        points[1][0].plot(t_data, data_pe)
        points[1][0].set_title("eci pe")
        points[2][0].plot(t_data, data_pd)
        points[2][0].set_title("eci pd")
        points[2][0].set(xlabel="time (s)")
        points[0][1].plot(t_data, data_pt_tor)
        points[0][1].set_title("orbital position pt")
        points[1][1].plot(t_data, data_po_tor)
        points[1][1].set_title("orbital position po")
        points[2][1].plot(t_data, data_pr_tor)
        points[2][1].set_title("orbital position pr")
        points[2][1].set(xlabel="time (s)")

        #speeds
        fig, speeds = plt.subplots(3, 2, sharex='all')
        speeds[0][0].plot(t_data, data_u)
        speeds[0][0].set_title("body u")
        speeds[1][0].plot(t_data, data_v)
        speeds[1][0].set_title("body v")
        speeds[2][0].plot(t_data, data_w)
        speeds[2][0].set_title("body w")
        speeds[2][0].set(xlabel="time (s)")
        speeds[0][1].plot(t_data, data_velt_tor)
        speeds[0][1].set_title("orbital velocity t")
        speeds[1][1].plot(t_data, data_velo_tor)
        speeds[1][1].set_title("orbital velocity o")
        speeds[2][1].plot(t_data, data_velr_tor)
        speeds[2][1].set_title("orbital velocity r")
        speeds[2][1].set(xlabel="time (s)")

        #attitude
        fig, att = plt.subplots(3, 2, sharex='all')
        att[0][0].plot(t_data, data_pitch)
        att[0][0].set_title("eci pitch")
        att[1][0].plot(t_data, data_roll)
        att[1][0].set_title("eci roll")
        att[2][0].plot(t_data, data_yaw)
        att[2][0].set_title("eci yaw")
        att[2][0].set(xlabel="time (s)")
        att[0][1].plot(t_data, data_pitch_tor)
        att[0][1].set_title("orbital pitch")
        att[1][1].plot(t_data, data_roll_tor)
        att[1][1].set_title("orbital roll")
        att[2][1].plot(t_data, data_yaw_tor)
        att[2][1].set_title("orbital yaw")
        att[2][1].set(xlabel="time (s)")

        fig, rates = plt.subplots(3, 2, sharex='all')
        rates[0][0].plot(t_data, data_p)
        rates[0][0].set_title("eci p")
        rates[1][0].plot(t_data, data_q)
        rates[1][0].set_title("eci q")
        rates[2][0].plot(t_data, data_r)
        rates[2][0].set_title("eci r")
        rates[2][0].set(xlabel="time (s)")
        rates[0][1].plot(t_data, data_yawdot_tor)
        rates[0][1].set_title("orbital yawdot")
        rates[1][1].plot(t_data, data_pitchdot_tor)
        rates[1][1].set_title("orbital pitch dot")
        rates[2][1].plot(t_data, data_rolldot_tor)
        rates[2][1].set_title("orbital roll dot")
        rates[2][1].set(xlabel="time (s)")


    elif args.returnECIdata:
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


    elif args.returnTORdata:
        # position orbital
        fig, points = plt.subplots(3, 1, sharex='all')
        points[0].plot(t_data, data_pt_tor)
        points[0].set_title("orbital position pt")
        points[1].plot(t_data, data_po_tor)
        points[1].set_title("orbital position po")
        points[2].plot(t_data, data_pr_tor)
        points[2].set_title("orbital position pr")
        points[2].set(xlabel="time (s)")

        # orbital velocities
        fig, speeds = plt.subplots(3, 1, sharex='all')
        speeds[0].plot(t_data, data_velt_tor)
        speeds[0].set_title("orbital velocity t")
        speeds[1].plot(t_data, data_velo_tor)
        speeds[1].set_title("orbital velocity o")
        speeds[2].plot(t_data, data_velr_tor)
        speeds[2].set_title("orbital velocity r")
        speeds[2].set(xlabel="time (s)")

        # orbital pitch roll yaw
        fig, att = plt.subplots(3, 1, sharex='all')
        att[0].plot(t_data, data_pitch_tor)
        att[0].set_title("orbital pitch")
        att[1].plot(t_data, data_roll_tor)
        att[1].set_title("orbital roll")
        att[2].plot(t_data, data_yaw_tor)
        att[2].set_title("orbital yaw")
        att[2].set(xlabel="time (s)")

        # pitch roll yaw dots tor
        fig, rates = plt.subplots(3, 1, sharex='all')
        rates[0].plot(t_data, data_yawdot_tor)
        rates[0].set_title("orbital yawdot")
        rates[1].plot(t_data, data_pitchdot_tor)
        rates[1].set_title("orbital pitch dot")
        rates[2].plot(t_data, data_rolldot_tor)
        rates[2].set_title("orbital roll dot")
        rates[2].set(xlabel="time (s)")

    # speed and alt
    fig, speed = plt.subplots(2, 1, sharex='all')
    speed[0].plot(t_data, data_speed)
    speed[0].set_title("speed")
    speed[1].plot(t_data, data_altitude, label="actual altitude")
    speed[1].plot(t_data, data_set_alt, label="set orbit altitude")
    speed[1].set_title("altitude")
    speed[1].legend()
    speed[1].set(xlabel="time (s)")

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
args.startPn = 400e3 + VPC.radius_e
args.gravityCntrl = 1
runTest(args)