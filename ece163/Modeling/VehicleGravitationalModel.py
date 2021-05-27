"""
Authors: Orbiting Satellite Group

This class defines the Gravitational model for the vehicle, which includes the gravity model, the forces
on the vehicle,
"""

import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleGravitationalModel():
    def __init__(self):
        """
        Initialization of the internal classes which are used to track the vehicle gravitational dynamics and dynamics.

        Parameters
        none

        Returns
        none
        """
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

        return

    def calculateThrustersForces(self, ThrusterX, ThrusterY, ThrusterZ):
        """
        This function calculates the forces on the satellite from the thrusters

        Parameters
        ThrusterX,Y,Z - Thruster Inputs [0-1]

        Returns
        thruster forces, a forcesMoments class
        """

        return

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



