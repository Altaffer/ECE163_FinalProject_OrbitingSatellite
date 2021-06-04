"""
Authors: Orbiting Satellite Group

This class defines the Gravitational model for the vehicle, which includes the gravity model, the forces
on the vehicle,
"""
import pymap3d as pm
import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
from ..Modeling import DisturbancesModel as dist

class VehicleGravitationalModel():
    def __init__(self, initialNorth=VPC.InitialNorth, initialEast=VPC.InitialEast, initialDown=VPC.InitialDown,
                 initialSpeed=VPC.InitialSpeed, gravity = True, controls = True, disturbances = True):
        """
        Initialization of the internal classes which are used to track the vehicle gravitational dynamics and dynamics.

        Parameters
        initialNorth - initial north position of the vehicle [m]
        initialEast - initial east position of the vehicle [m]
        initialSouth - initial south position of the vehicle [m]

        Returns
        none
        """
        #instantiate initial positions
        self.initialNorth = initialNorth
        self.initialEast = initialEast
        self.initialDown = initialDown
        self.initialSpeed = initialSpeed

        #create a dynamics model to function on
        self.VehicleDynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel(initialNorth=self.initialNorth,
                                                                              initialEast=self.initialEast,
                                                                              initialDown=self.initialDown,
                                                                              initialSpeed=self.initialSpeed)
        # instantiate kwargs for isolating VGM features for tests
        self.gravity = gravity
        self.controls = controls
        self.disturbances = disturbances

        return

    def gravityForces(self, state):
        """
        Unlike the plane, this model assumes that we will have a really high altitude, so gravity changes to a more
        complex model

        Parameters
        state – current vehicle state (need the rotation matrix)

        Returns
        gravity forces, forcesMoments class
        """

        #create a class to return
        gravityForces = Inputs.forcesMoments()

        # Gravity is aligned with the vector facing from the satellite to the center of the earth
        sat_to_earth_vec = [[-state.pn], [-state.pe], [-state.pd]]
        ste_mag = math.hypot(-state.pn, -state.pe, -state.pd)

        #using the gravity model, equation 6.5 in Giancoli, 4.2 in Fortesque
        Fg = VPC.G * (VPC.mass * VPC.mass_e) / ((ste_mag) ** 2)

        # the vector of unit lenght one pointing from the satellite to earth
        sat_to_earth_norm = mm.scalarDivide(ste_mag, sat_to_earth_vec)

        # force of gravity pointing towards the earth from the satellite
        Fg_inertial = mm.scalarMultiply(Fg, sat_to_earth_norm)

        #rotate into the satellite body frame
        Fg_body = mm.multiply(Rotations.euler2DCM(state.yaw, state.pitch, state.roll), Fg_inertial)

        #now partition the body frame forces into the class to return
        gravityForces.Fx = Fg_body[0][0]
        gravityForces.Fy = Fg_body[1][0]
        gravityForces.Fz = Fg_body[2][0]

        return gravityForces

    def calculateThrustersForces(self, ThrusterX, ThrusterY, ThrusterZ):
        """
        This function calculates the forces on the satellite from the thrusters

        Parameters
        ThrusterX,Y,Z - Thruster Inputs [0-1]

        Returns
        thruster forces, a forcesMoments class
        """
        # return class
        thrustersForces = Inputs.forcesMoments()

        # the thrusters are already aligned with the body frame, so we can directly set the Fx, Fy, and Fz
        # Fx from thruster x
        if ThrusterX == 0:
            thrustersForces.Fx = 0  # corner case where if the control is zero, the thrust is zero
        elif ThrusterX < 0:
            thrustersForces.Fx = (VPC.C_thruster*ThrusterX) - VPC.Thruster_min  # corner case for negative thrust
        else:
            thrustersForces.Fx = (VPC.C_thruster * ThrusterX) + VPC.Thruster_min  # standard thrust equation (from VPC)

        # Fy from thruster y
        if ThrusterY == 0:
            thrustersForces.Fy = 0  # corner case where if the control is zero, the thrust is zero
        elif ThrusterY < 0:
            thrustersForces.Fy = (VPC.C_thruster*ThrusterY) - VPC.Thruster_min  # corner case for negative thrust
        else:
            thrustersForces.Fy = (VPC.C_thruster * ThrusterY) + VPC.Thruster_min  # standard thrust equation (from VPC)

        # Fz from thruster z
        if ThrusterZ == 0:
            thrustersForces.Fz = 0  # corner case where if the control is zero, the thrust is zero
        elif ThrusterZ < 0:
            thrustersForces.Fz = (VPC.C_thruster*ThrusterZ) - VPC.Thruster_min  # corner case for negative thrust
        else:
            thrustersForces.Fz = (VPC.C_thruster * ThrusterZ) + VPC.Thruster_min  # standard thrust equation (from VPC)


        # we are also assuming that the thrusters are perfectly aligned with Center of Mass so they introduce
        # no moment to the satellite

        return thrustersForces

    def calculateReactionWheelForces(self, ReactionX, ReactionY, ReactionZ):
        """
        Function to calculate the moments from the reaction wheels

        Parameters
        ReactionX,Y,Z - the reaction wheel control inputs [0-1]

        Returns
        reaction wheel forces, a forces moments class
        """
        # return class
        reactionWheelForces = Inputs.forcesMoments()

        # Because the reaction wheels are on axis, the moments Mx, My, and Mz are just functions of the control input
        reactionWheelForces.Mx = VPC.C_reaction * ReactionX
        reactionWheelForces.My = VPC.C_reaction * ReactionY
        reactionWheelForces.Mz = VPC.C_reaction * ReactionZ

        return reactionWheelForces

    def disturbanceForces(self, state):
        """
        Function to calculate the disturbance forces from the disturbance model, should be a simple conversion from
        acclerations in the disturbance model to forces

        Parameters
        state – current vehicle state

        Returns
        disturbance forces, forcesMoments class
        """
        # class to return
        disturbanceForces = Inputs.forcesMoments()

        #get direction from disturbance model and multiply by force, (m*a)
        sunGravForce = mm.scalarMultiply(VPC.mass * VPC.sunAcc, dist.distanceFromSun(state))
        moonGravForce = mm.scalarMultiply(VPC.mass * VPC.moonAcc, dist.distanceFromMoon(state))
        jupGravForce = mm.scalarMultiply(VPC.mass * VPC.jupAcc, dist.distanceFromJupiter(state))

        #get the surface area projected perpendicular to the sun and multiply by the acceleration and the direction
        radiationForce = mm.scalarMultiply(VPC.radiationAcc * dist.satSurfaceArea(state),
                                           mm.scalarMultiply(-1, dist.distanceFromSun(state)))

        #fill forces moments model
        disturbanceForces.Fx = sunGravForce[0][0] + moonGravForce[0][0] + jupGravForce[0][0] + radiationForce[0][0]
        disturbanceForces.Fy = sunGravForce[1][0] + moonGravForce[1][0] + jupGravForce[1][0] + radiationForce[1][0]
        disturbanceForces.Fz = sunGravForce[2][0] + moonGravForce[2][0] + jupGravForce[2][0] + radiationForce[2][0]

        return disturbanceForces

    def updateForces(self, state, controls):
        """
        Function to update all of the disturbance, propulsive, and gravity forces and moments. All calculations
        required to update the forces are included.

        Parameters
        state – current vehicle state
        controls – current vehicle controls

        Returns
        total forces, forcesMoments class
        """
        #class to return
        totalForces = Inputs.forcesMoments()

        #update grav, thrust, reaction, and disturbance forces and add to total forces. Check kwargs
        if self.gravity:
            totalForces = totalForces + self.gravityForces(state)
        if self.controls:
            totalForces = totalForces + self.calculateThrustersForces(controls.ThrusterX, controls.ThrusterY,
                                                                      controls.ThrusterZ) + \
                          self.calculateReactionWheelForces(controls.ReactionX, controls.ReactionY, controls.ReactionZ)
        if self.disturbances:
            totalForces = totalForces + self.disturbanceForces(state)

        return totalForces

    def Update(self, controls):
        """
        Function that uses the current state (internal), disturbance model (internal), and controls (inputs) to
        calculate the forces, and then do the integration of the full 6-DOF non-linear equations of motion. Wraps the
        VehicleDynamicsModel class as well as the disturbanceModel internally.

        Parameters
        controls – controlInputs class

        Returns
        none, state is updated internally
        """
        # use the vehicle dynamics model instance state to update the forces
        newForces = self.updateForces(self.VehicleDynamicsModel.state, controls)

        # use the new forces to update the dynamics model instance, and therefore the current state
        self.VehicleDynamicsModel.Update(newForces)

        return

    def reset(self):
        """
        Resets module to its original state so it can run again

        Returns
        none
        """
        #reset the vehicle dynamics model
        self.VehicleDynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel(initialNorth=self.initialNorth,
                                                                              initialEast=self.initialEast,
                                                                              initialDown=self.initialDown,
                                                                              initialSpeed=self.initialSpeed)
        return

    def getVehicleDynamicsModel(self):
        """
        Wrapper function to return the vehicle dynamics model handle

        Returns
        vehicleDynamicsModel, from VehicleDynamicsModel class
        """
        return self.VehicleDynamicsModel

    def getVehicleState(self):
        """
        Wrapper function to return vehicle state form module

        Returns
        vehicle state class
        """
        return self.VehicleDynamicsModel.state

    def setVehicleState(self, state):
        """
        Wrapper function to set the vehicle state from outside module

        Parameters
        state – class of vehicleState

        Returns
        none
        """
        self.VehicleDynamicsModel.state = state

        return



