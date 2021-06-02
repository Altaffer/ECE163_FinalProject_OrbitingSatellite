"""
Authors: Orbiting Satellite Group

This file implements the closed loop control system of the spacecraft
"""

import math
from ..Containers import Inputs
from ..Containers import Controls
# from ..Constants import VehiclePhysicalConstants as VPC
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Utilities import OrbitalFrame as of

vpcdT = 1/100

class PControl():
    def __init__(self, kp=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        #initialize keyword arguments
        self.kp = kp
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        return

    def Update(self, command=0.0, current=0.0):
        #calculate the error
        error = command - current

        #calculate u
        u = self.trim + (self.kp * error)

        #check if u is saturated
        if u > self.highLimit:
            u = self.highLimit
        elif u < self.lowLimit:
            u = self.lowLimit

        return u

    def setPGains(self, kp=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        #assign kwargs to set the gains
        self.kp = kp
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

class PDControl():
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Functions which implement the PD control with saturation where the derivative is available as a separate input
        to the function. The output is: u = u_ref + Kp * error - Kd * dot{error} limited between lowLimit and highLimit.

        Parameters
        kp – proportional gain
        kd – derivative gain
        trim – trim output (added the the loop computed output)
        lowLimit – lower limit to saturate control
        highLimit – upper limit to saturate control

        Returns
        none
        """
        #initialize keyword arguments
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """
        Calculates the output of the PD loop given the gains and limits from instantiation, and using the command,
        actual, and derivative inputs. Output is limited to between lowLimit and highLimit from instantiation.

        Parameters
        command – reference command
        current – actual output (or sensor)
        derivative – derivative of the output or sensor

        Returns
        u [control] limited to saturation bounds
        """
        #calculate the error
        error = command - current

        #calculate u
        u = self.trim + (self.kp * (error)) - (self.kd * derivative)

        #check if u is saturated
        if u > self.highLimit:
            u = self.highLimit
        elif u < self.lowLimit:
            u = self.lowLimit

        return u

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Function to set the gains for the PD control block (including the trim output and the limits)

        Parameters
        kp – proportional gain
        kd – derivative gain
        trim – trim output (added the the loop computed output)
        lowLimit – lower limit to saturate control
        highLimit – upper limit to saturate control

        Returns
        none
        """
        #assign kwargs to set the gains
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        return

class PIControl():
    def __init__(self, dT=vpcdT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Functions which implement the PI control with saturation where the integrator has both a reset and an
        anti-windup such that when output saturates, the integration is undone and the output forced the output to the
        limit. The output is: u = u_ref + Kp * error + Ki * integral{error} limited between lowLimit and highLimit.

        Parameters
        dT – time step [s], required for integration
        kp – proportional gain
        ki – integral gain
        trim – trim input
        lowLimit – low saturation limit
        highLimit – high saturation limit

        Returns
        none
        """

        #initialize and assign kwargs
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        #initialize placeholder for previous error
        self.error_prev = 0.0

        #initialize an accumulator
        self.acc = 0

        return

    def Update(self, command = 0.0, current = 0.0):
        """
        Calculates the output of the PI loop given the gains and limits from instantiation, and using the command and
        current or actual inputs. Output is limited to between lowLimit and highLimit from instantiation. Integration
        for the integral state is done using trapezoidal integration, and anti-windup is implemented such that if the
        output is out of limits, the integral state is not updated (no additional error accumulation).

        Parameters
        command – reference command
        current – current output or sensor

        Returns
        u [output] limited to saturation bounds
        """
        #calculate current error
        error = command - current


        # PERFORM INTEGRAL OF ERROR
        #integrate error over recent timestep using trapezoidal integration
        error_integral = (self.dT * (error + self.error_prev)) / 2

        #integrate by use of accumulator - add recent timestep to sum of all timesteps
        self.acc += error_integral


        #calculate u
        u = self.trim + (self.kp * error) + (self.ki * self.acc)

        #check saturation and perform anti-windup
        if u > self.highLimit:
            u = self.highLimit
            self.acc -= error_integral # anti windup prevents integration if in saturation
        elif u < self.lowLimit:
            u = self.lowLimit
            self.acc -= error_integral  # anti windup prevents integration if in saturation

        #store previous error value
        self.error_prev = error

        return u

    def resetIntegrator(self):
        """
        Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral
        state.

        Returns
        none
        """

        #reset accumulator
        self.acc = 0

        #reset previous error
        self.error_prev = 0

        return

    def setPIGains(self, dT=vpcdT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Function to set the gains for the PI control block (including the trim output and the limits)

        Parameters
        dT – time step [s], required for integration
        kp – proportional gain
        kd – derivative gain
        ki – integral gain
        trim – trim input
        lowLimit – low saturation limit
        highLimit – high saturation limit

        Returns
        none
        """

        #reset all parameter values to kwargs
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        return

class PIDControl():
    def __init__(self, dT=vpcdT, kp=0.0, ki=0.0, kd = 0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Functions which implement the PID control with saturation where the integrator has both a reset and an
        anti-windup such that when output saturates, the integration is undone and the output forced the output to the
        limit. Function assumes that physical derivative is available (e.g.: roll and p), not a numerically derived one.
        The output is: u = u_ref + Kp * error - Kd * dot{error} + Ki * integral{error} limited between lowLimit and
        highLimit.

        Parameters
        dT – time step [s], required for integration
        kp – proportional gain
        ki – integral gain
        kd - derivative gain
        trim – trim input
        lowLimit – low saturation limit
        highLimit – high saturation limit

        Returns
        none
        """

        #initialize and assign kwargs
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        #initialize placeholder for previous error
        self.error_prev = 0.0

        #initialize an accumulator
        self.acc = 0

        return

    def Update(self, command = 0.0, current = 0.0, derivative=0.0):
        """
        Calculates the output of the PID loop given the gains and limits from instantiation, and using the command and
        current or actual inputs. Output is limited to between lowLimit and highLimit from instantiation. Integration
        for the integral state is done using trapezoidal integration, and anti-windup is implemented such that if the
        output is out of limits, the integral state is not updated (no additional error accumulation).

        Parameters
        command – reference command
        current – current output or sensor
        derivative – derivative of the output or sensor

        Returns
        u [output] limited to saturation bounds
        """
        #calculate current error
        error = command - current


        # PERFORM INTEGRAL OF ERROR
        #integrate error over recent timestep
        error_integral = (self.dT * (error + self.error_prev)) / 2

        #integrate by use of accumulator - add recent timestep to sum of all timesteps
        self.acc += error_integral


        #calculate u
        u = self.trim + (self.kp * error) + (self.ki * self.acc) - (self.kd * derivative)

        #check saturation and perform anti-windup
        if u > self.highLimit:
            u = self.highLimit
            self.acc -= error_integral # anti windup prevents integration if in saturation
        elif u < self.lowLimit:
            u = self.lowLimit
            self.acc -= error_integral  # anti windup prevents integration if in saturation

        #store previous error value
        self.error_prev = error

        return u

    def resetIntegrator(self):
        """
        Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral
        state.

        Returns
        none
        """

        #reset accumulator
        self.acc = 0

        #reset previous error
        self.error_prev = 0

        return

    def setPIDGains(self, dT=vpcdT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Function to set the gains for the PID control block (including the trim output and the limits)

        Parameters
        dT – time step [s], required for integration
        kp – proportional gain
        kd – derivative gain
        ki – integral gain
        trim – trim input
        lowLimit – low saturation limit
        highLimit – high saturation limit

        Returns
        none
        """

        #reset all parameter values to kwargs
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        return

class VehicleClosedLoopControl():
    def __init__(self,  dT=0.01, OrbitVector = [[0],[0],[-400000]]):
                                                        #magnitude of vector is equivalent to orbit radius
        """
        Class that implements the entire closed loop control

        Parameters
        none

        Returns
        none
        """

        self.OrbitVector = OrbitVector

        self.dT = dT

        self.thrustersFromVTangent = PIControl()

        self.VOffsetFromOffset = PDControl()
        self.thrustersFromVoffset = PControl()

        self.VRadialFromRadial = PDControl()
        self.thrustersFromVRadial = PControl()

        self.rollDotFromRoll = PIDControl()
        self.pitchDotFromPitch = PIDControl()
        self.yawDotFromYaw = PIDControl()

        self.reactorXfromP = PControl()
        self.reactorYfromQ = PControl()
        self.reactorZfromR = PControl()

        return

    def setControlGains(self):
        self.thrustersFromVTangent.setPIGains(dT=self.dT, kp = 0, ki=0, lowLimit=-1, highLimit=1)
        
        self.VOffsetFromOffset.setPDGains(kp=0, kd=0, lowLimit=-100, highLimit=100)
        self.thrustersFromVoffset.setPGains(kp=0, lowLimit=-1, highLimit=1)

        self.VRadialFromRadial.setPDGains(kp=0, kd=0, lowLimit=-100, highLimit=100)
        self.thrustersFromVRadial.setPGains(kp=0, lowLimit=-1, highLimit=1)

        self.rollDotFromRoll.setPIDGains(dT=self.dT, kp=0,kd=0,ki=0, lowLimit=-3.14, highLimit=3.14)
        self.pitchDotFromPitch.setPIDGains(dT=self.dT, kp=0,kd=0,ki=0, lowLimit=-3.14, highLimit=3.14)
        self.yawDotFromYaw.setPIDGains(dT=self.dT, kp=0,kd=0,ki=0, lowLimit=-3.14, highLimit=3.14)

        self.reactorXfromP.setPGains(kp=0, lowLimit=-1, highLimit=1)
        self.reactorYfromQ.setPGains(kp=0, lowLimit=-1, highLimit=1)
        self.reactorZfromR.setPGains(kp=0, lowLimit=-1, highLimit=1)

    def reset(self):
        self.thrustersFromVTangent.resetIntegrator()
        self.rollDotFromRoll.resetIntegrator()
        self.pitchDotFromPitch.resetIntegrator()
        self.yawDotFromYaw.resetIntegrator()

    def UpdateControlCommands(self, vehicleState:States.vehicleState):
        R_e2o, R_o2e = of.orbitalFrameR(self.OrbitVector, vehicleState)

        # Getting Position in Orbital Frame
        ECI_Pos = [[vehicleState.pn], [vehicleState.pe], [vehicleState.pd]]
        ORB_Pos = mm.multiply(R_e2o, ECI_Pos)

        # Getting Velocity in Orbital Frame
        Body_Vel = [[vehicleState.u], [vehicleState.v], [vehicleState.w]]
        R_b2e = mm.transpose(vehicleState.R)
        ECI_Vel = mm.multiply(R_b2e, Body_Vel)
        ORB_Vel = mm.multiply(R_e2o, ECI_Vel)

        # Getting Yaw, Pitch, Roll in Inertial frame
        yaw = vehicleState.yaw
        pitch = vehicleState.pitch
        roll = vehicleState.roll

        # Getting Yaw, Pitch, Roll dot in inertial frame
        p = vehicleState.p
        q = vehicleState.q
        r = vehicleState.r
        
        pqr = mm.transpose([[p, q, r]])
        weird_matrix = [[1, math.sin(roll)*math.tan(pitch), math.cos(roll)*math.tan(pitch)],
                        [0, math.cos(roll), -math.sin(roll)],
                        [0, math.sin(roll)/math.cos(pitch), math.cos(roll)/math.cos(pitch)]]
        rollpitchyaw_dot = mm.multiply(weird_matrix, pqr)
        rollDot, pitchDot, yawDot = mm.transpose(rollpitchyaw_dot)[0]


        # Getting Commanded Radius
        rc = math.hypot(self.OrbitVector[0][0], self.OrbitVector[1][0], self.OrbitVector[2][0])

        # Getting Commanded Velocity
        a = 9.7 # TODO set to actual acceleration with respect to radius
        VTan_command = math.sqrt(a*rc)

        # Getting Commanded Yaw, Pitch, Roll in Inertial Frame
        yawc, pitchc, rollc = Rotations.dcm2Euler(R_e2o)

        # Getting thruster command along T axis
        T_ThrusterCommand = self.thrustersFromVTangent.Update(VTan_command, ORB_Vel[0][0])

        # getting thruster command along O axis
        OffsetVelCommand = self.VOffsetFromOffset.Update(0, ORB_Pos[1][0], ORB_Vel[1][0])
        O_ThrusterCommand = self.thrustersFromVoffset.Update(OffsetVelCommand, ORB_Vel[1][0])

        # getting thruster command along R axis
        RadialVelCommand = self.VRadialFromRadial.Update(rc,ORB_Pos[2][0], ORB_Vel[2][0])
        R_ThrusterCommand = self.thrustersFromVRadial.Update(RadialVelCommand, ORB_Vel[2][0])

        # Getting change in yaw, pitch, roll commands
        yawDotCommand = self.yawDotFromYaw.Update(yawc, yaw, yawDot)
        pitchDotCommand = self.pitchDotFromPitch.Update(pitchc, pitch, pitchDot)
        rollDotCommand = self.rollDotFromRoll.Update(rollc, roll, rollDot)

        # Getting commanded body angular velocity commands
        eulerDotCommand = [[rollDotCommand], [pitchDotCommand], [yawDotCommand]]
        weird_matrix_2 = [[1,0,math.sin(pitch)],\
                          [0, math.cos(roll), math.sin(roll)*math.cos(pitch)],\
                          [0,-math.sin(roll), math.cos(roll)*math.cos(pitch)]]
        pqr_command = mm.multiply(weird_matrix_2,eulerDotCommand)
        pCommand, qCommand, rCommand = mm.transpose(pqr_command)[0]

        # Getting Reaction wheel commands
        reactorXcommand = self.reactorXfromP.Update(pCommand, p)
        reactorYcommand = self.reactorYfromQ.Update(qCommand, q)
        reactorZcommand = self.reactorZfromR.Update(rCommand, r)



    def Update(self):
        """
        Function that updates the control system

        Parameters
        none

        Returns
        None
        """

        return


