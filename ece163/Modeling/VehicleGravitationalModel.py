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

"""
Editing Notes:
5/28 5:00pm - Richie - adding gravityForces and init - UNTESTED so may be trash atm
"""

class VehicleGravitationalModel():
    def __init__(self, initialNorth=VPC.InitialNorth, initialEast=VPC.InitialEast, initialDown=VPC.InitialDown):
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

        #create a dynamics model to function on
        self.VehicleDynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel(initialNorth=self.initialNorth,
                                                                              initialEast=self.initialEast,
                                                                              initialDown=self.initialDown)

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

        #using the gravity model, equation 6.5 in Giancoli, 4.2 in Fortesque
        Fg = VPC.G * (VPC.mass * VPC.mass_e) / ((-state.pd + VPC.radius_e) ** 2)    #-pd + rad_earth should be distance
                                                                                    #from center of masses

        #Gravity is aligned with the vector facing from the satellite to the center of the earth
        sat_to_earth_vec = [[-state.pn],[-state.pe], [-state.pd]]
        ste_mag = math.hypot(state.pn, state.pe, state.pd)

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

        return

    def disturbanceForces(self, state):
        """
        Function to calculate the disturbance forces from the disturbance model, should be a simple conversion from
        acclerations in the disturbance model to forces

        Parameters
        state – current vehicle state

        Returns
        disturbance forces, forcesMoments class
        """
        return

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
        return

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
        return

    def reset(self):
        """
        Resets module to its original state so it can run again

        Returns
        none
        """
        return

    def getVehicleDynamicsModel(self):
        """
        Wrapper function to return the vehicle dynamics model handle

        Returns
        vehicleDynamicsModel, from VehicleDynamicsModel class
        """
        return

    def getVehicleState(self):
        """
        Wrapper function to return vehicle state form module

        Returns
        vehicle state class
        """
        return

    def setVehicleState(self, state):
        """
        Wrapper function to set the vehicle state from outside module

        Parameters
        state – class of vehicleState

        Returns
        none
        """
        return



