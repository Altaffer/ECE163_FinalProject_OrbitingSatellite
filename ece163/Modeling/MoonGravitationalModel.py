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

class MoonGravitationalModel():
    def __init__(self, initialNorth=VPC.MoonInitialNorth, initialEast=VPC.MoonInitialEast,
                 initialDown=VPC.MoonInitialDown, initialU=VPC.MoonInitialU, initialV=VPC.MoonInitialV,
                 initialW=VPC.MoonInitialW, gravity = True):
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
        self.initialU = initialU
        self.initialV = initialV
        self.initialW = initialW

        #create a dynamics model to function on
        self.MoonDynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel(initialNorth=self.initialNorth,
                                                                              initialEast=self.initialEast,
                                                                              initialDown=self.initialDown,
                                                                              initialU = self.initialU,
                                                                              initialV = self.initialV,
                                                                              initialW = self.initialW)
        # instantiate kwargs for isolating MGM features for tests
        self.gravity = gravity

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

        # Gravity is aligned with the vector facing from the moon to the center of the earth
        moon_to_earth_vec = [[-state.pn], [-state.pe], [-state.pd]]
        ste_mag = math.hypot(-state.pn, -state.pe, -state.pd)

        #using the gravity model, equation 6.5 in Giancoli, 4.2 in Fortesque
        Fg = VPC.G * (VPC.mass_m * VPC.mass_e) / ((ste_mag) ** 2)

        # the vector of unit length one pointing from the moon to earth
        moon_to_earth_norm = mm.scalarDivide(ste_mag, moon_to_earth_vec)

        # force of gravity pointing towards the earth from the moon
        Fg_inertial = mm.scalarMultiply(Fg, moon_to_earth_norm)

        #rotate into the moon body frame
        Fg_body = mm.multiply(Rotations.euler2DCM(state.yaw, state.pitch, state.roll), Fg_inertial)

        #now partition the body frame forces into the class to return
        gravityForces.Fx = Fg_body[0][0]
        gravityForces.Fy = Fg_body[1][0]
        gravityForces.Fz = Fg_body[2][0]

        return gravityForces

    def updateForces(self, state):
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
        return totalForces

    def Update(self):
        """
        Function that uses the current state (internal) to
        calculate the forces, and then do the integration of the full 6-DOF non-linear equations of motion. Wraps the
        VehicleDynamicsModel class as well as the disturbanceModel internally.

        Parameters
        controls – controlInputs class

        Returns
        none, state is updated internally
        """
        # use the vehicle dynamics model instance state to update the forces
        newForces = self.updateForces(self.MoonDynamicsModel.state)

        # use the new forces to update the dynamics model instance, and therefore the current state
        self.MoonDynamicsModel.Update(newForces)

        return

    def reset(self):
        """
        Resets module to its original state so it can run again

        Returns
        none
        """
        #reset the vehicle dynamics model
        self.MoonDynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel(initialNorth=self.initialNorth,
                                                                              initialEast=self.initialEast,
                                                                              initialDown=self.initialDown,
                                                                              initialU = self.initialU,
                                                                              initialV = self.initialV,
                                                                              initialW = self.initialW)
        return

    def getMoonDynamicsModel(self):
        """
        Wrapper function to return the vehicle dynamics model handle

        Returns
        moonDynamicsModel, from VehicleDynamicsModel class
        """
        return self.MoonDynamicsModel

    def getMoonState(self):
        """
        Wrapper function to return vehicle state form module

        Returns
        vehicle state class
        """
        return self.MoonDynamicsModel.state

    def setMoonState(self, state):
        """
        Wrapper function to set the vehicle state from outside module

        Parameters
        state – class of vehicleState

        Returns
        none
        """
        self.MoonDynamicsModel.state = state

        return



