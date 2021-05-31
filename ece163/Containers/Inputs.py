"""
File contains the classes that define the inputs to the various parts of the simulator. These classes are used
internally, and more will be added as needed. This file is differentiated from the States.py file which contains the
classes that are internal states tracked within the simulator.
"""

import math

testingAbs_tol = 1e-6

class forcesMoments:
	def __init__(self, Fx=0.0, Fy=0.0, Fz=0.0, Mx=0.0, My=0.0, Mz=0.0):
		"""
		Defines the forces [N] and moments [N-m] struct such that this can be passed around to the various functions that need to
		use them. Forces and moments are defined in the body-frame and assumed to be located at the center of mass.

		:param Fx: sum of forces in body-x direction [N]
		:param Fy: sum of forces in body-y direction [N]
		:param Fz: sum of forces in body-z direction [N]
		:param Mx: sum of moments about body-x direction [N-m]
		:param My: sum of moments about body-y direction [N-m]
		:param Mz: sum of moments about body-z direction [N-m]
		"""
		self.Fx = Fx
		self.Fy = Fy
		self.Fz = Fz
		self.Mx = Mx
		self.My = My
		self.Mz = Mz
		return

	def __repr__(self):
		return "{0.__name__}(Fx={1.Fx}, Fy={1.Fy}, Fz={1.Fz}, Mx={1.Mx}, My={1.My}, Mz={1.Mz})".format(type(self), self)

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all(
					[math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol) for member in ['Fx', 'Fy', 'Fz',
																								'Mx', 'My', 'Mz']]):
				return False
			else:
				return True
		else:
			return NotImplemented

	def __add__(self, other):
		if isinstance(other, type(self)):
			sum = forcesMoments()
			sum.Fx = self.Fx + other.Fx
			sum.Fy = self.Fy + other.Fy
			sum.Fz = self.Fz + other.Fz

			sum.Mx = self.Mx + other.Mx
			sum.My = self.My + other.My
			sum.Mz = self.Mz + other.Mz

			return sum
		else:
			print("ERROR: Trying to add forcesMoments object and non-forcesMoments object")
			return NotImplemented


class controlInputs:
	def __init__(self, ThrusterX=0.5, ThrusterY=0.0, ThrusterZ=0.0, ReactionX=0.0, ReactionY=0.0, ReactionZ=0.0):
		"""
		A container for the control inputs to the thrusters and reaction wheel setup. Currently for a 3-axis thruster
		system and 3-axis reaction wheel system.

		Parameters
		ThrusterX - control input for the thruster in the body x direction [0-1]
		ThrusterY - control input for the thruster in the body y direction [0-1]
		ThrusterZ - control input for the thruster in the body z direction [0-1]
		ReactionX - control input for the reaction wheel about the body x direction [0-1]
		ReactionY - control input for the reaction wheel about the body y direction [0-1]
		ReactionZ - control input for the reaction wheel about the body z direction [0-1]

		"""
		self.ThrusterX = ThrusterX
		self.ThrusterY = ThrusterY
		self.ThrusterZ = ThrusterZ

		self.ReactionX = ReactionX
		self.ReactionY = ReactionY
		self.ReactionZ = ReactionZ
		return