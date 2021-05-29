"""
Author: Richard Owens (rivowens@ucsc.edu)
This module is where all of the vehicle dynamics are computed for the simulation. It includes the kinematics of
both the translational and rotational dynamics. Included are both the derivative, and the integration functions,
and the rotations of forces to the body frame.
"""

import math
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

"""
Editing Notes:
5/28 5:00pm - Richie - added initial NED to Init and to Reset - should work fine
"""

class VehicleDynamicsModel():
    def __init__(self, dT=VPC.dT, initialNorth=VPC.InitialNorth, initialEast=VPC.InitialEast, initialDown=VPC.InitialDown):
        """
        def __init__(self, dT=VPC.dT): Initializes the class, and sets the time step (needed for Rexp and integration).
        Should keep track of the state and the state derivative internally.

        Parameters
        dT – defaults to VPC.dT
        initialNorth - initial north position of the vehicle [m]
        initialEast - initial east position of the vehicle [m]
        initialSouth - initial south position of the vehicle [m]

        Returns
        none
        """
        #initialize initial position
        self.initialNorth = initialNorth
        self.initialEast = initialEast
        self.initialDown = initialDown

        #default dt to 0.01
        self.dT = dT

        #default states
        self.state = States.vehicleState(pn=self.initialNorth, pe=self.initialEast, pd=initialDown)

        #default derivatives
        self.dot = States.vehicleState()

        return

    def ForwardEuler(self, dT, state, dot):
        """
        def ForwardEuler(self, forcesMoments): Function to do the simple forwards integration of the state using the
        derivative function. State is integrated using the x_{k+1} = x_{k} + dx/dt * dT. The updated state is returned
        by the function. The state and derivative are held internally as members to the class.

        Parameters
        dT – the timestep over which to forward integrate
        state – the initial state to integrate, as an instance of State.vehicleState
        dot – the time-derivative of the state for performing integration, as an instance of State.vehicleState

        Returns
        new state, advanced by a timestep of dT (defined in States.vehicleState class)
        """

        #create the state to be returned
        newState = States.vehicleState()

        #integrate points
        newState.pn = state.pn + (dot.pn * dT)
        newState.pe = state.pe + (dot.pe * dT)
        newState.pd = state.pd + (dot.pd * dT)

        # integrate velocities
        newState.u = state.u + (dot.u * dT)
        newState.v = state.v + (dot.v * dT)
        newState.w = state.w + (dot.w * dT)

        # integrate rotation rates
        newState.p = state.p + (dot.p * dT)
        newState.q = state.q + (dot.q * dT)
        newState.r = state.r + (dot.r * dT)

        return newState

    def IntegrateState(self, dT, state, dot):
        """
        Updates the state given the derivative, and a time step. Attitude propagation is implemented as a DCM matrix
        exponential solution, all other state params are advanced via forward euler integration [x]k+1 = [x]k + xdot*dT.
        The integrated state is returned from the function. All derived variables in the state (e.g.: Va, alpha, beta,
        chi) should be copied from the input state to the returned state.

        Parameters
        dT – Time step [s]
        state – the initial state to integrate, as an instance of State.vehicleState
        dot – the time-derivative of the state for performing integration, as an instance of State.vehicleState

        Returns
        new state, advanced by a timestep of dT, returned as an instance of the States.vehicleState class
        """

        #create state to be returned
        newState = States.vehicleState()

        #update R
        newState.R = mm.multiply(self.Rexp(dT, state, dot), state.R)

        #use forward euler for the rest
        forwardState = self.ForwardEuler(dT, state, dot) #create a state that stores forward returns

        newState.pn = forwardState.pn #store points
        newState.pe = forwardState.pe
        newState.pd = forwardState.pd

        newState.u = forwardState.u #store velocities
        newState.v = forwardState.v
        newState.w = forwardState.w

        newState.p = forwardState.p #store rotation rates
        newState.q = forwardState.q
        newState.r = forwardState.r

        #derive yaw pitch and roll from dcm
        ypr = Rotations.dcm2Euler(newState.R)
        newState.yaw = ypr[0]
        newState.pitch = ypr[1]
        newState.roll = ypr[2]

        #apply alpha, beta, va, and chi
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2(dot.pe, dot.pn)

        return newState

    def Rexp(self, dT, state, dot):
        """
        Calculates the matrix exponential exp(-dT*[omega x]), which can be used in the closed form solution for the DCM
        integration from body-fixed rates.

        See the document (ECE163_AttitudeCheatSheet.pdf) for details.

        Parameters
        dT – time step [sec]
        state – the vehicle state, in the form of a States.vehicleState object
        dot – the state derivative, in the form of a States.vehicleState object

        Returns
        Rexp: the matrix exponential to update the state
        """
        #perform trapezoidal approximation for omega
        p_trap = state.p + (0.5 * dot.p * dT)
        q_trap = state.q + (0.5 * dot.q * dT)
        r_trap = state.r + (0.5 * dot.r * dT)

        #create a magnitude/norm for w for calculations
        omega_mag = math.sqrt((p_trap**2) + (q_trap**2) + (r_trap**2))

        #create an omega cross matrix for calculations, along with an identity matrix
        I = [[1,0,0],
             [0,1,0],
             [0,0,1]]

        omega_x = [[0, -r_trap, q_trap],
                   [r_trap, 0, -p_trap],
                   [-q_trap, p_trap, 0]]

        #find norm of omega_x * dt - sqrt(sum of all elements squared
        norm = math.sqrt((2*((p_trap * dT)**2)) + (2*((q_trap*dT)**2)) + (2*((r_trap*dT)**2)))

        # if || dT*omega x || is less than 0.2, use the maclauren approximation
        if norm >= 0.02:
            sinFunc = math.sin(omega_mag * dT) / omega_mag  # from equation 22
            cosFunc = (1 - math.cos(omega_mag * dT)) / (omega_mag ** 2)  # from equation 22
        else:
            sinFunc = dT - (((dT ** 3) * (omega_mag ** 2)) / 6) + (((dT ** 5) * (omega_mag ** 4)) / 120)  # from eqn 24
            cosFunc = ((dT ** 2) / 2) - (((dT ** 4) * (omega_mag ** 2)) / 24) + (
                        ((dT ** 6) * (omega_mag ** 4)) / 720)  # from eqn 24

        # solve for exp - from equation 22
        RexpFirstTwo = mm.subtract(I, mm.scalarMultiply(sinFunc, omega_x))
        Rexp = mm.add(RexpFirstTwo, mm.scalarMultiply(cosFunc, mm.multiply(omega_x, omega_x)))

        return Rexp

    def Update(self, forcesMoments):
        """
        Function that implements the integration such that the state is updated using the forces and moments passed in
        as the arguments (dT is internal from the member). State is updated in place (self.state is updated). Use
        getVehicleState to retrieve state. Time step is defined in VehiclePhyscialConstants.py

        Parameters
        forcesMoments – forces [N] and moments [N-m] defined in forcesMoments class

        Returns
        none
        """

        #update the derivative of the state
        self.dot = self.derivative(self.state, forcesMoments)

        #update current state with the integral of the derivative
        self.state = self.IntegrateState(self.dT, self.state, self.dot)

        return

    def derivative(self, state, forcesMoments):
        """
        Function to compute the time-derivative of the state given body frame forces and moments

        Parameters
        state – state to differentiate, as a States.vehicleState object
        forcesMoments – forces [N] and moments [N-m] as an Inputs.forcesMoments object

        Returns
        the current time derivative, in the form of a States.vehicleState object
        """

        #the vehicle state to be returned
        der = States.vehicleState()

        #derive the position points - equation 5
        velocity = [[state.u], [state.v], [state.w]]

        d_pos = mm.multiply(mm.transpose(state.R), velocity)
        der.pn = d_pos[0][0]
        der.pe = d_pos[1][0]
        der.pd = d_pos[2][0]

        #derive the velocity points - equation 1
        omega_x = [[0, -state.r, state.q], #omega cross used in this and future functions
                   [state.r, 0, -state.p],
                   [-state.q, state.p, 0]]
        Fb = [[forcesMoments.Fx], [forcesMoments.Fy], [forcesMoments.Fz]]

        d_vel = mm.subtract(mm.scalarMultiply(1/VPC.mass, Fb),
                            mm.multiply(omega_x, velocity))

        der.u = d_vel[0][0]
        der.v = d_vel[1][0]
        der.w = d_vel[2][0]

        # derive the dcm - equation 19
        if state.p == 0 and state.q == 0 and state.r == 0:
            der.R = [[1,0,0],
                     [0,1,0],
                     [0,0,1]]
        else:
            der.R = mm.multiply(mm.scalarMultiply(-1, omega_x), state.R)

        #derive the euler angles equation 16
        omega = [[state.p], [state.q], [state.r]]
        Euler_Rot = [[1, math.sin(state.roll) * math.tan(state.pitch), math.cos(state.roll) * math.tan(state.pitch)],
                     [0, math.cos(state.roll), -math.sin(state.roll)],
                     [0, math.sin(state.roll) / math.cos(state.pitch), math.cos(state.roll) / math.cos(state.pitch)]]

        d_ypr = mm.multiply(Euler_Rot, omega)

        der.yaw = d_ypr[2][0]
        der.pitch = d_ypr[1][0]
        der.roll = d_ypr[0][0]

        #derive the rotation rates - equation 9
        Mb = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]]

        d_pqr = mm.multiply(VPC.JinvBody, mm.subtract(Mb, mm.multiply(omega_x, mm.multiply(VPC.Jbody, omega))))

        der.p = d_pqr[0][0]
        der.q = d_pqr[1][0]
        der.r = d_pqr[2][0]

        return der

    def getVehicleDerivative(self):
        """
        Getter method to read the vehicle state time derivative

        Returns
        dot ( an instance of Containers.States.vehicleState)
        """

        return self.dot

    def getVehicleState(self):
        """
        Getter method to read the vehicle state

        Returns
        state (from class vehicleState)
        """

        return self.state

    def reset(self):
        """
        Reset the Vehicle state to initial conditions

        Returns
        none
        """

        #reset all values in the self.state
        self.state.pn = self.initialNorth
        self.state.pe = self.initialEast
        self.state.pd = self.initialDown
        self.state.u = 0.0
        self.state.v = 0.0
        self.state.w = 0.0
        self.state.p = 0.0
        self.state.q = 0.0
        self.state.r = 0.0
        self.state.yaw = 0.0
        self.state.pitch = 0.0
        self.state.roll = 0.0
        self.state.R = [[1,0,0],
                        [0,1,0],
                        [0,0,1]]

        # apply alpha, beta, va, and chi
        self.state.alpha = 0.0
        self.state.beta = 0.0
        self.state.Va = 0.0
        self.state.chi = 0.0

        return

    def setVehicleDerivative(self, dot):
        """
        Setter method to write the vehicle state time derivative

        Parameters
        state – dot to be set (should be an instance of Containers.States.vehicleState)

        Returns
        none
        """

        self.dot = dot

        return

    def setVehicleState(self, state):
        """
        Setter method to write the vehicle state

        Parameters
        state – state to be set ( an instance of Containers.States.vehicleState)

        Returns
        none
        """

        self.state = state
        return



