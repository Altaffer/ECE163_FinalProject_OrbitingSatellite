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
from matplotlib import pyplot as plt

# TODO orbital position pt is broken, maybe from rotations?

# TO RUN TESTS/SIMULATIONS, SCROLL TO THE BOTTOM TO INPUT A TEST

class testArgs():
    def __init__(self, dT=1, time=90*60, sample_duration=15,
                 startOrbitalSpeed=7700, orbitVector=[[0],[0],[-(400000+VPC.radius_e)]], \
                 orbitStartPosNED=[[0], [400000+VPC.radius_e], [0]],  controlGains = VCLC.ControlGains(), \
                 gravityCntrl=0, controlsCntrl=0, disturbancesCntrl=0, returnECIdata=0, returnTORdata=0,
                 returnCntrlData=0, returnAnimationNE=0, returnAnimationED=0, returnAnimationND=0):
        self.dT = dT  # time step between each plotted point in seconds
        # this is different from the VGM time step
        # a dT of 1 would have 100 physics steps
        # between each plotted point if the VGM dT = .01
        self.time = time  # duration of the simulation in seconds
        self.sample_duration = sample_duration  # time between samples for data collection
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
        self.returnCntrlData = returnCntrlData  # set to 1 to return plots of the controls
        # set to zero to ignore
        self.returnAnimationNE = returnAnimationNE  # set to 1 to return an animation of NE plane
        # set to zero to ignore
        self.returnAnimationED = returnAnimationED  # set to 1 to return an animation of ED plane
        # set to zero to ignore
        self.returnAnimationND = returnAnimationED  # set to 1 to return an animation of ND plane
        # set to zero to ignore

        #to find the initial speed of the craft in uvw to initialize the VGM, rotate to orbital frame speed to body
        state = States.vehicleState(pn=self.orbitStartPosNED[0][0], pe=self.orbitStartPosNED[1][0],
                                    pd=self.orbitStartPosNED[2][0])  # create a temporary state with NED to get rots
        Reci2orbital, Rorbital2eci = OF.orbitalFrameR(self.orbitVector, state)  # get rots for eci and orbit
        Rbody2orbital, Rorbital2body = OF.getBodyOrbitalRotationMatrices(Reci2orbital, Rorbital2eci, state)  # get body rotations
        self.speedUVW = mm.multiply(Rorbital2body, [[self.startOrbitalSpeed], [0], [0]])  # get the uvw speed from orbit

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

    gravModel.getVehicleDynamicsModel().dT = args.dT

    clControl = VCLC.VehicleClosedLoopControl(dT=args.dT, OrbitVector=args.orbitVector)
    clControl.setControlGains(args.controlGains)

    # GRAPH VARIOUS STATE VALUES OVER time SECONDS
    # define time steps and total time
    dT = args.dT
    T_tot = args.time
    sampleSize = args.sample_duration
    n_steps = int((T_tot / dT) / sampleSize)

    # define datasets
    t_data = [(i * dT * sampleSize) for i in range(n_steps)]

    # state data
    data_pn = [0 for i in range(n_steps)]
    data_pe = [0 for i in range(n_steps)]
    data_pd = [0 for i in range(n_steps)]
    if args.returnECIdata:
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
    if args.returnCntrlData:
        data_thrusterX = [0 for i in range(n_steps)]
        data_thrusterY = [0 for i in range(n_steps)]
        data_thrusterZ = [0 for i in range(n_steps)]

    # speed/ altitude data
    data_speed = [0 for i in range(n_steps)]
    data_altitude = [0 for i in range(n_steps)]
    data_set_alt = [math.hypot(args.orbitVector[0][0], args.orbitVector[1][0], args.orbitVector[2][0]) - VPC.radius_e
                    for i in range(n_steps)]

    # FILL DATASETS
    # update repeatedly over time interval
    for i in range(int(n_steps * sampleSize)):
        # record data - FILL WITH THE SPECIFIC DATA YOU ARE LOOKING FOR,
        # ex: data[i] = gravModel.getVehicleState().pd   for the height

        # get state data
        if i % sampleSize == 0:
            a = int(i/sampleSize)
            data_pn[a] = gravModel.getVehicleState().pn
            data_pe[a] = gravModel.getVehicleState().pe
            data_pd[a] = gravModel.getVehicleState().pd

            if args.returnECIdata:

                data_u[a] = gravModel.getVehicleState().u
                data_v[a] = gravModel.getVehicleState().v
                data_w[a] = gravModel.getVehicleState().w

                data_pitch[a] = gravModel.getVehicleState().pitch
                data_roll[a] = gravModel.getVehicleState().roll
                data_yaw[a] = gravModel.getVehicleState().yaw

                data_p[a] = gravModel.getVehicleState().p
                data_q[a] = gravModel.getVehicleState().q
                data_r[a] = gravModel.getVehicleState().r

            # to get data in TOR, we need to rotate the above values
            if args.returnTORdata:
                Re2o, Ro2e = OF.orbitalFrameR(args.orbitVector, gravModel.getVehicleState())  # get rotation vectors
                Orb_pos, Orb_vel = OF.getOrbitalAxisVals(Re2o, Ro2e, gravModel.getVehicleState()) # get position & vel in ORB

                data_pt_tor[a] = Orb_pos[0][0]
                data_po_tor[a] = Orb_pos[1][0]
                data_pr_tor[a] = Orb_pos[2][0]

                data_velt_tor[a] = Orb_vel[0][0]
                data_velo_tor[a] = Orb_vel[1][0]
                data_velr_tor[a] = Orb_vel[2][0]

                data_yaw_tor[a], data_pitch_tor[a], data_roll_tor[a], data_yawdot_tor[a], data_pitchdot_tor[a], \
                data_rolldot_tor[a] = OF.getOrbitalAngularVals(Re2o, Ro2e, gravModel.getVehicleState())

            # get speed/location data
            data_speed[a] = math.hypot(gravModel.getVehicleState().u, gravModel.getVehicleState().v,
                                       gravModel.getVehicleState().w)
            data_altitude[a] = math.hypot(gravModel.getVehicleState().pn, gravModel.getVehicleState().pe,
                                          gravModel.getVehicleState().pd) - VPC.radius_e

        # update the controls model
        controls = clControl.UpdateControlCommands(gravModel.getVehicleState())

        if i % sampleSize == 0:
            a = int(i/sampleSize)
            if args.returnCntrlData:
                data_thrusterX[a] = controls.ThrusterX
                data_thrusterY[a] = controls.ThrusterY
                data_thrusterZ[a] = controls.ThrusterZ

        # update the gravitational model
        gravModel.Update(controls)



    # PLOT DATA
    if args.returnAnimationNE:
        # animation for NE plane
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
            # collect data
            x_data.append(data_pe[i] / 1e6)
            y_data.append(data_pn[i] / 1e6)

            if sampleSize > 20:
                # calculate days elapsed
                if hours == 24:
                    days += 1
                    hours = 0
                # calculate hours elapsed
                if mins >= 60:
                    hours += 1
                    mins -= 60
                mins += (args.dT * sampleSize) / 60
            else:
                # calculate days elapsed
                if hours == 24:
                    days += 1
                    hours = 0
                # calculate hours elapsed
                if mins == 60:
                    hours += 1
                    mins = 0
                # calculate minutes elapsed
                if secs >= 60:
                    mins += 1
                    secs -= 60
                secs += args.dT*sampleSize
            simtime += 1

            if i % 3 == 0:
                # plot earth, set graph limits, set title, plot data
                plt.gca().add_patch(earth)
                plt.xlim(-20, 20)
                plt.ylim(-20, 20)
                plt.plot(x_data, y_data, color='blue', linewidth=1)
                plt.title(
                    "Days elapsed: {days}   Hours elapsed: {hours}   Mins elapsed: {mins},    SimTime = {simtime} samples" \
                    .format(days=days, hours=hours, mins=mins, simtime=simtime))
                plt.xlabel("Position east (million meters)")
                plt.ylabel("Position north (million meters)")
                plt.pause(0.001)
        plt.show()

    if args.returnAnimationND:
        # animation for NE plane
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
            # collect data
            x_data.append(data_pn[i] / 1e6)
            y_data.append(data_pd[i] / 1e6)

            if sampleSize > 20:
                # calculate days elapsed
                if hours == 24:
                    days += 1
                    hours = 0
                # calculate hours elapsed
                if mins >= 60:
                    hours += 1
                    mins -= 60
                mins += (args.dT * sampleSize) / 60
            else:
                # calculate days elapsed
                if hours == 24:
                    days += 1
                    hours = 0
                # calculate hours elapsed
                if mins == 60:
                    hours += 1
                    mins = 0
                # calculate minutes elapsed
                if secs >= 60:
                    mins += 1
                    secs -= 60
                secs += args.dT * sampleSize
            simtime += 1

            if i % 3 == 0:
                # plot earth, set graph limits, set title, plot data
                plt.gca().add_patch(earth)
                plt.xlim(-20, 20)
                plt.ylim(20, -20)
                plt.plot(x_data, y_data, color='blue', linewidth=1)
                plt.title(
                    "Days elapsed: {days}   Hours elapsed: {hours}   Mins elapsed: {mins},    SimTime = {simtime} samples" \
                    .format(days=days, hours=hours, mins=mins, simtime=simtime))
                plt.xlabel("Position east (million meters)")
                plt.ylabel("Position north (million meters)")
                plt.pause(0.001)
        plt.show()

    if args.returnAnimationED:
        # animation for NE plane
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
            # collect data
            x_data.append(data_pe[i] / 1e6)
            y_data.append(data_pd[i] / 1e6)

            if sampleSize > 20:
                # calculate days elapsed
                if hours == 24:
                    days += 1
                    hours = 0
                # calculate hours elapsed
                if mins >= 60:
                    hours += 1
                    mins -= 60
                mins += (args.dT * sampleSize) / 60
            else:
                # calculate days elapsed
                if hours == 24:
                    days += 1
                    hours = 0
                # calculate hours elapsed
                if mins == 60:
                    hours += 1
                    mins = 0
                # calculate minutes elapsed
                if secs >= 60:
                    mins += 1
                    secs -= 60
                secs += args.dT * sampleSize
            simtime += 1

            if i % 5 == 0:
                # plot earth, set graph limits, set title, plot data
                plt.gca().add_patch(earth)
                plt.xlim(-20, 20)
                plt.ylim(20, -20)
                plt.plot(x_data, y_data, color='blue', linewidth=1)
                plt.title("Days elapsed: {days}   Hours elapsed: {hours}   Mins elapsed: {mins},    SimTime = {simtime} samples" \
                          .format(days=days, hours=hours, mins=mins, simtime=simtime))
                plt.xlabel("Position east (million meters)")
                plt.ylabel("Position down (million meters)")
                plt.pause(0.001)
        plt.show()

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

    if args.returnCntrlData:
        # thruster controls
        fig, thrust = plt.subplots(3, 1, sharex='all')
        thrust[0].plot(t_data, data_thrusterX)
        thrust[0].set_title("thruster x")
        thrust[1].plot(t_data, data_thrusterY)
        thrust[1].set_title("thruster y")
        thrust[2].plot(t_data, data_thrusterZ)
        thrust[2].set_title("thruster z")
        thrust[2].set(xlabel="time (s)")
        plt.show()

    return


# A given test can be run by constructing a testArgs class and passing it into the runTest function
# For a description of what each parameter does, look to the top of the file to view the testArgs class
# parameter descriptions
# An example test would be placing the satellite at an orbit of 400km, turning off the controls and disturbances
# and observing as gravity causes it to plummet towards the earth

#SPEED AND POSITION VECTORS MUST BE PUT INSIDE INIT AS THEY ARE USED WITHIN THE INITIALIZATION
def high_alt_orbit_test_highSpeed():
    # pretty decent NE plan, high altitude orbit test. Starting speed is too high and would throw us off orbit if
    # not for controls

    args = testArgs(startOrbitalSpeed=7200,
                    orbitVector=[[0], [0], [-(4e6 + VPC.radius_e)]],
                    orbitStartPosNED=[[0], [4e6 + VPC.radius_e], [0]])
    args.returnAnimationNE = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 3 * 60 * 90
    args.sample_duration = 3 * 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)

    # ^^^ Best positional gains so far
    runTest(args)
    return

def high_alt_orbit_test_lowSpeed():
    # pretty decent NE plan, high altitude orbit test. Starting speed is too low now

    args = testArgs(startOrbitalSpeed=6000,
                    orbitVector=[[0], [0], [-(4e6 + VPC.radius_e)]],
                    orbitStartPosNED=[[0], [4e6 + VPC.radius_e], [0]])
    args.returnAnimationNE = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 3 * 60 * 90
    args.sample_duration = 3 * 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)
    runTest(args)
    return

def ISS_orbit_test():
    # ISS speed and altitude test, not much here to control the uncontrolled orbit is pretty on point bc of physics
    # but a good example of a realistic orbit and start settings
    args = testArgs(startOrbitalSpeed=7700,
                    orbitVector=[[0],[0],[-(4e5+VPC.radius_e)]],
                    orbitStartPosNED=[[0], [4e5+VPC.radius_e], [0]])
    args.returnAnimationNE = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 60*90
    args.sample_duration = 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)
    runTest(args)
    return

def EDplane_orbit_test():
    # this is the high altitude orbit test but turned 90 degrees
    args = testArgs(startOrbitalSpeed=7200,
                    orbitVector=[[4e6 + VPC.radius_e], [0], [0]],
                    orbitStartPosNED=[[0], [4e6 + VPC.radius_e], [0]])
    args.returnAnimationNE = 0
    args.returnAnimationED = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 3 * 60 * 90
    args.sample_duration = 3 * 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)

    # ^^^ Best positional gains so far
    runTest(args)
    return

def inclined_high_alt_orbit():
    #same altitude as before now on an incline of 45 degrees
    args = testArgs(startOrbitalSpeed=7200,
                    orbitVector=[[7333411], [0], [7333411]],
                    orbitStartPosNED=[[0], [4e6 + VPC.radius_e], [0]])
    args.returnAnimationNE = 1
    args.returnAnimationND = 1
    args.returnAnimationED = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 3 * 60 * 90
    args.sample_duration = 3 * 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)

    runTest(args)
    return

#THIS TEST IS OVER A LONG TIME PERIOD AND TAKES A SEC TO RUN
def off_altitude_orbit_startLow():
    # in this test the sat starts off altitude and finds the wanted altitude

    percent_alt = 0.95
    args = testArgs(startOrbitalSpeed=7200,
                    orbitVector=[[0], [0], [-(4e6 + VPC.radius_e)]],
                    orbitStartPosNED=[[0], [percent_alt * (4e6 + VPC.radius_e)], [0]])
    args.returnAnimationNE = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 10 * 60 * 90
    args.sample_duration = 10 * 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)

    runTest(args)
    return

# THIS TEST IS OVER A LONG TIME PERIOD AND TAKES A SEC TO RUN
def off_altitude_orbit_startHigh():
    # in this test the sat starts off altitude and finds the wanted altitude

    percent_alt = 1.05
    args = testArgs(startOrbitalSpeed=7200,
                    orbitVector=[[0], [0], [-(4e6 + VPC.radius_e)]],
                    orbitStartPosNED=[[0], [percent_alt * (4e6 + VPC.radius_e)], [0]])
    args.returnAnimationNE = 1
    args.returnECIdata = 0
    args.returnTORdata = 0
    args.gravityCntrl = 1
    args.controlsCntrl = 1
    args.returnCntrlData = 1
    args.time = 10 * 60 * 90
    args.sample_duration = 10 * 15
    args.controlGains = VCLC.ControlGains(Vtan_kp=0.1, Vtan_ki=0.1, Offset_kp=0.1, Offset_kd=0.0,
                                          Voffset_kp=0.1, Radial_kp=0.1, Radial_kd=0.1, Vradial_kp=0.1)

    runTest(args)
    return

high_alt_orbit_test_highSpeed()

