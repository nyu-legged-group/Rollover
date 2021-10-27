from __future__ import division
import crocoddyl
import pinocchio
import numpy as np
from pinocchio.utils import eye, rotate, zero
from generator import SE3Generator
from itertools import cycle
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter, AutoMinorLocator)
from visualizer import ReferenceState
class SimpleHumanoidGaitProblem:
	""" Defines a 3d falling locomotion problem
	"""
	def __init__(self, rmodel, q0, torso, lFoot, rFoot,lKneePoint, rKneePoint,rKneePoint1, lHandPoint, rHandPoint):
		self.rmodel = rmodel
		self.rdata = rmodel.createData()
		self.state = crocoddyl.StateMultibody(self.rmodel)
		self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)

		# Getting the frame id for all the legs
		self.rfId = self.rmodel.getFrameId(rFoot)
		self.lfId = self.rmodel.getFrameId(lFoot)
		self.rkId = self.rmodel.getFrameId(rKneePoint)
		self.rk1Id = self.rmodel.getFrameId(rKneePoint1)
		self.lkId = self.rmodel.getFrameId(lKneePoint)
		self.rhId = self.rmodel.getFrameId(rHandPoint)
		self.lhId = self.rmodel.getFrameId(lHandPoint)
		self.torsoId = self.rmodel.getFrameId(torso)

		# Defining default state
		self.rmodel.defaultState = np.concatenate([q0, np.zeros((self.rmodel.nv, 1))])
		self.s = ReferenceState(self.rmodel.defaultState)
		self.firstStep = True

		# Defining the friction coefficient and normal
		self.mu = 0.7
		self.nsurf = np.matrix([0., 0., 1.]).T

	def createBimanualFallProblem(self, x0, handLength, comLength, timeStep, groundKnots, flyingKnots, final=False):
		# print(x0[27:])
		q0 = x0[:self.rmodel.nq]
		pinocchio.forwardKinematics(self.rmodel, self.rdata, q0)
		pinocchio.updateFramePlacements(self.rmodel, self.rdata)
		rkPos0 = self.rdata.oMf[self.rkId].translation
		lkPos0 = self.rdata.oMf[self.lkId].translation
		rhPos0 = self.rdata.oMf[self.rhId].translation
		lhPos0 = self.rdata.oMf[self.lhId].translation
		comRef = pinocchio.centerOfMass(self.rmodel, self.rdata, q0)
		self.rWeight = 1e1
		comRefNew = comRef + comLength
		comRefLow = comRefNew-0.01
		comRefHigh = comRefNew + 0.01
		comTask = []
		comTask.append(comRefLow)
		comTask.append(comRefHigh)
		comTask.append(comRefNew)
		# print(comTask)
		loco3dModel = []

		# 1. Falling down phase: knee contact
		fallingDownPhase = []
		flyingKnots =20
		groundKnots = 30

		# flyingKnots = 10
		# groundKnots = 70

		s = self.s   

		# s.value['r_knee'] = [+np.pi/2., 1000.]
		s.value['l_hip_y'][1] = 500.
		s.value['l_hip_r'][1] = 500.
		# s.value['l_hip_p'][1] = 500.
		s.value['l_ankle_p'][1] = 500.
		s.value['l_ankle_r'][1] = 500.

		s.value['r_hip_y'][1] = 500.
		s.value['r_hip_r'][1] = 500.
		# s.value['r_hip_p'][1] = 500.
		s.value['r_knee'][1] = 1000.
		s.value['l_knee'][1] = 1000.
		s.value['r_hip_p'][1] = 1000.
		s.value['l_hip_p'][1] = 1000.
		
		s.value['r_ankle_p'][1] = 500.
		s.value['r_ankle_r'][1] = 500.
		# s.value['r_shoulder_p'][1] = 100.
		# s.value['l_shoulder_p'][1] = 100.
		# s.value['r_shoulder_r'][1] = 100.
		# s.value['l_shoulder_r'][1] = 100.
		# # s.value['r_shoulder_y'][1] = 100.
		# # s.value['l_shoulder_y'][1] = 100.
		# s.value['r_elbow'][1] = 100.
		# s.value['l_elbow'][1] = 100.
		# s.value['x'][1] = 10.
		# s.value['y'][1] = 10.
		# s.value['z'][1] = 10.
		# s.value['rx'][1] = 10.
		# s.value['ry'][1] = 10.
		# s.value['rz'][1] = 10.
		
		




		s.update()
		stateWeight = []
		stateWeight.append(s.reference_state)
		stateWeight.append(s.stateWeights)

		for k in range(flyingKnots):
			fallingDownPhase += [self.createSwingFootModel(timeStep, [self.lkId, self.rkId], stateWeight)]

		# 2. Landing phase: knee contact and hand contact
		# handLength[0]-=0.03
		
		f0 =  handLength.copy()
		f1 =  handLength.copy()
		f0[0] -=0.14
		f1[0] -=0.14
		f1[1] *=-1
		# print(lhPos0, rhPos0)
		# print(lhPos0+f1)
		# print(rhPos0+f0)

		contactTask = [
			crocoddyl.FramePlacement(self.lkId, pinocchio.SE3(np.eye(3), lkPos0 )),
			crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0 )),
			crocoddyl.FramePlacement(self.lhId, pinocchio.SE3(np.eye(3), lhPos0+f1)),
			crocoddyl.FramePlacement(self.rhId, pinocchio.SE3(np.eye(3), rhPos0+f0))
		]
		landingPhase = [self.createFootSwitchModel([self.lkId, self.rkId, self.lhId, self.rhId], contactTask, False)]
		
		s.value['l_shoulder_p'] = [-73./180.*np.pi, 100]
		s.value['r_shoulder_p'] = [-73./180.*np.pi, 100]
		s.value['l_shoulder_r'] = [25./180.*np.pi,100]
		s.value['r_shoulder_r'] = [25./180.*np.pi,100]

		# 3. Landed phase: knee contact and hand contact
		s.value['r_elbow'] = [(180.-131.)/180.*np.pi, 100.]
		s.value['l_elbow'] = [(180.-131.)/180.*np.pi, 100.]
		s.value['r_shoulder_y'] = [np.pi/2, 100.]
		s.value['l_shoulder_y'] = [-np.pi/2, 100.]
		# s.value['r_shoulder_p'] = [0, 1.]
		# s.value['r_shoulder_p'] = [0, 1.]

		s.update()
		stateWeight = []
		stateWeight.append(s.reference_state)
		stateWeight.append(s.stateWeights)

		if final is True:
			self.rWeight = 1e4
		f1 = comLength
		landed = [
			self.createSwingFootModel(timeStep, [self.lkId, self.rkId, self.lhId, self.rhId], stateWeight)
			for k in range(groundKnots)
		]

		# 4. Assemble all phases
		loco3dModel += fallingDownPhase
		loco3dModel += landingPhase
		loco3dModel += landed

		problem = crocoddyl.ShootingProblem(x0, loco3dModel, loco3dModel[-1])
		return problem
	
	def createRolloverFallProblem(self, x0, handLength, timeLength, timeStep, groundKnots, flyingKnots, final=False):

		q0 = x0[:self.rmodel.nq]
		pinocchio.forwardKinematics(self.rmodel, self.rdata, q0)
		pinocchio.updateFramePlacements(self.rmodel, self.rdata)
		rkPos0 = self.rdata.oMf[self.rkId].translation
		rk1Pos0 = self.rdata.oMf[self.rk1Id].translation
		lkPos0 = self.rdata.oMf[self.lkId].translation
		rhPos0 = self.rdata.oMf[self.rhId].translation
		lhPos0 = self.rdata.oMf[self.lhId].translation
		rfPos0 = self.rdata.oMf[self.rfId].translation
		lfPos0 = self.rdata.oMf[self.lfId].translation
		
		comLower = rkPos0[1]
		comUpper = rk1Pos0[1]
		comRef = pinocchio.centerOfMass(self.rmodel, self.rdata, q0)

		comTask = []
		comTask.append(comLower)
		comTask.append(comUpper)
		comTask.append(comRef)

		f0 =  handLength
		
		f0[1]+=0.05
		f0[0]+=0.13
		self.rWeight = 1e1

		loco3dModel = []
		# self.rmodel.defaultState[8] = 0
		# 1. Falling down phase: knee contact
		fallingDownPhase = []
		rhPos = rhPos0 +f0

		# Torso task
		SE3 = pinocchio.SE3
		start_SE3 = SE3(eye(3), np.matrix([0, 0, 0]).T)*SE3(eye(3), np.matrix([0, 0, 0]).T)
		mid_SE3 = start_SE3*SE3(rotate('y', np.pi/6),zero(3))*SE3(rotate('z', np.pi/2),zero(3))
		# Foot task 
		foot_se3 = start_SE3*SE3(rotate('x', +np.pi/2),zero(3))
		# end_SE3 = mid_SE3 * SE3(rotate('z', np.pi/2),zero(3))
		# se3FlyingGenerator = SE3Generator(start_SE3, mid_SE3, sample_length=(flyingKnots))
		# se3GroundGenerator = SE3Generator(mid_SE3, end_SE3, sample_length=(groundKnots))
		# torsoFlyingTask = se3FlyingGenerator.traj
		# torsoGroundTask 
		# . 
		# = se3GroundGenerator.traj
		
		moving_com_knot = timeLength[0]
		falling_knot = timeLength[1]
		rotating_knot = timeLength[2]

		# moving_com_knot = 30
		# falling_knot = 14
		# rotating_knot = 27

		s = self.s
		####Phase1
		################################## LEFT LEG##################################
		s.value['r_hip_y'] = [-np.pi/6, 100.]
		s.value['r_hip_r'] = [np.pi/4, 100.]
		s.value['l_hip_r'] = [10./180.*np.pi, 100.]
		s.value['l_hip_y'] = [0., 100.]
		
		# s.value['l_hip_p'] = [-np.pi/6, 100.]
		
		################################## RIGHT LEG##################################
		# s.value['r_hip_r'] = [-np.pi/4, 100.]
		# s.value['l_hip_r'] = [+np.pi/4, 100.]
		
		s.value['r_knee'] = [-np.pi/2-np.pi/10, 100.]
		s.value['l_knee'] = [-np.pi/2-np.pi/10, 100.]
		# s.value['x'][1] = s.value['y'][1] = s.value['z'][1] = 10.
		# s.value['rx'][1] = s.value['ry'][1] = s.value['rz'][1] = 10.
		# s.value['l_hip_p'] = [np.pi/3,100.]
		# s.value['r_hip_p'] = [np.pi/3, 100.]
		# s.value['l_knee'] = [-np.pi/2., 100.]
		################################## LEFT ARM##################################

		# Folding left arm
		s.value['l_shoulder_p'] = [-np.pi/4, 100.]
		s.value['l_shoulder_r'] = [np.pi/18, 100.]
		s.value['l_shoulder_y'] = [-np.pi/2, 100.]
		s.value['l_elbow'] = [np.pi/2+np.pi/6, 100.]
		################################## RIGHT ARM##################################
		# Keeping forward right arm
		s.value['r_shoulder_y'] = [np.pi/2+np.pi/2,100.]
		s.value['r_shoulder_r'] = [-np.pi/18, 100.]
		s.value['r_shoulder_p'][1] = 100.
		s.value['r_elbow'][1] = 100.
		
		# s.value['r_knee'] = [+np.pi/2., 1000.]

		s.update()
		stateWeight = []
		stateWeight.append(s.reference_state)
		stateWeight.append(s.stateWeights)
		for k in range(moving_com_knot):
			swingFootTask = []
			swingFootTask += [crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0))]
			swingFootTask += [crocoddyl.FramePlacement(self.lkId, pinocchio.SE3(np.eye(3), lkPos0))]
			fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId,self.lkId], stateWeight, swingFootTask=swingFootTask)]

		s.value['r_elbow'][1] = 1.
		s.update()
		stateWeight = []
		stateWeight.append(s.reference_state)
		stateWeight.append(s.stateWeights)
		for k in range(falling_knot):
			swingFootTask = []
			swingFootTask += [crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0))]
			fallingDownPhase += [self.createSwingFootModel1(timeStep, [self.rkId], stateWeight, swingFootTask=swingFootTask)]
		
		contactTask = [
			crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0 )),
			crocoddyl.FramePlacement(self.rhId, pinocchio.SE3(np.eye(3), rhPos ))
		]
		
		fallingDownPhase += [self.createFootSwitchModel([self.rkId, self.rhId], contactTask, False)]

		####Phase2
		################################
		s.value['r_hip_y'] = [15./180.*np.pi, 100.]
		s.value['r_hip_r'] = [15./180.*np.pi, 100.]
		s.value['l_hip_r'] = [40./180.*np.pi, 100.]
		s.value['l_hip_y'] = [15./180.*np.pi, 100.]
		
		s.value['r_hip_p'] = [np.pi/4, 100.]
		# s.value['l_hip_p'] = [-np.pi/4, 100.]
		
		s.value['l_shoulder_p'] = [-np.pi/2, 100.]
		s.value['l_shoulder_r'] = [np.pi, 100.]
		s.value['l_elbow'] = [np.pi/6, 100.]
		# s.value['r_shoulder_p'] = [-np.pi/2.-np.pi/6., 100.]
		# s.value['r_shoulder_r'] = [-np.pi/10., 100.]
		# s.value['r_shoulder_y'] = [np.pi/2., 100.]
		s.value['r_elbow'] = [np.pi/2+np.pi/6, 100.]
		s.value['v_r_elbow'][1] = 10.
		# s.value['v_r_elbow'][1] = 0.
		# s.value['r_hip_y'][1] = 100.


		s.value['l_knee'] = [-np.pi/2., 100.]
		s.value['r_knee'][0] -=0.
		s.value['r_knee'][1] =1.
		s.value['v_rx'] = [0.1,10.]
		s.value['v_rz'] = [0.1,10.]

		s.update()
		stateWeight = []
		stateWeight.append(s.reference_state)
		# print(s.reference_state, s.stateWeights)
		# exit()
		stateWeight.append(s.stateWeights)

		for k in range(rotating_knot):
			swingFootTask = []
			swingFootTask += [crocoddyl.FramePlacement(self.rkId, foot_se3)]
			swingFootTask += [crocoddyl.FramePlacement(self.rhId, pinocchio.SE3(np.eye(3), rhPos))]
			fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], stateWeight, torsoTask =mid_SE3.rotation, swingFootTask=swingFootTask, collisionTask=[self.rfId], isTerminal=False)]
		# region
		# doubleContact = 2
		# flyingKnots = groundKnots
		# for k in range(flyingKnots):
		#     swingFootTask = []
		#     if k == flyingKnots-1:
		#         swingFootTask += [crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0))]
		#         swingFootTask += [crocoddyl.FramePlacement(self.rhId, pinocchio.SE3(np.eye(3), rhPos))]
		#         #swingFootTask += [crocoddyl.FramePlacement(self.rfId, pinocchio.SE3(np.eye(3), rfPos0))]
		#                         # comTask = comRef + f0/2/(flyingKnots-1)*k
		#         # fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], swingFootTask=swingFootTask, comTask=comTask, torsoTask =torsoFlyingTask[k].rotation)]
		#         # fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], swingFootTask=swingFootTask, torsoTask =torsoFlyingTask[k].rotation)]
		#         fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], k,torsoTask =mid_SE3.rotation, swingFootTask=swingFootTask, collisionTask=[self.rfId], isTerminal=True)]
		#     else:
		#         '''
		#         if k < doubleContact:
		#             swingFootTask += [crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0))]
		#             swingFootTask += [crocoddyl.FramePlacement(self.rk1Id, pinocchio.SE3(np.eye(3), rk1Pos0))]
		#             fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rk1Id], swingFootTask=swingFootTask)]
		#         else:
		#         '''
		#         # if k < doubleContact:
		#         #     swingFootTask += [crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0))]
		#         #     swingFootTask += [crocoddyl.FramePlacement(self.lkId, pinocchio.SE3(np.eye(3), lkPos0))]
		#         #     fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.lkId], k, comTask=comTask, swingFootTask=swingFootTask)]
		#         # else:
		#         swingFootTask += [crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0))]
		#         #fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId], comTask=comTask, swingFootTask=swingFootTask, collisionTask=[self.rfId])]
		#         fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId], k, comTask=comTask, swingFootTask=swingFootTask)]
		#         # comTask = comRef + f0/2/(flyingKnots-1)*k
		#         # fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], swingFootTask=swingFootTask, comTask=comTask, torsoTask =torsoFlyingTask[k].rotation)]
		#         # fallingDownPhase += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], swingFootTask=swingFootTask, torsoTask =torsoFlyingTask[k].rotation)]
				
		'''
		# 2. Landing phase: knee contact and hand contact
		

		contactTask = [
			crocoddyl.FramePlacement(self.rkId, pinocchio.SE3(np.eye(3), rkPos0 )),
			crocoddyl.FramePlacement(self.rhId, pinocchio.SE3(np.eye(3), rhPos ))
		]
		
		landingPhase = [self.createFootSwitchModel([self.rkId, self.rhId], contactTask, False)]

		if final is True:
			self.rWeight = 1e4
		f1 = np.matrix([0, -0.05,0]).T
		# landed = [
		#     self.createSwingFootModel(timeStep, [self.rkId, self.rhId], torsoTask =torsoGroundTask[k].rotation, comTask=comRef +f0/2 +f1/(groundKnots-1)*k)
		#     for k in range(groundKnots)
		# ]
		for k in range(groundKnots):
			landed = []
			if k == groundKnots-1:
				landed += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId], torsoTask =end_SE3.rotation, collisionTask=[self.lfId], isTerminal=True)]
			else:
				landed += [self.createSwingFootModel(timeStep, [self.rkId, self.rhId])]
		'''
		# endregion
		loco3dModel += fallingDownPhase
		#loco3dModel += landingPhase
		#loco3dModel += landed

		problem = crocoddyl.ShootingProblem(x0, loco3dModel, loco3dModel[-1])
		return problem
	
	def createSwingFootModel(self, timeStep, supportFootIds, stateWeight, comTask=None, swingFootTask=None, torsoTask =None, collisionTask = None, isTerminal = False):
		""" Action model for a swing foot phase.

		:param timeStep: step duration of the action model
		:param supportFootIds: Ids of the constrained feet
		:param comTask: CoM task
		:param swingFootTask: swinging foot task
		:return action model for a swing foot phase
		"""
		# Creating a 6D multi-contact model, and then including the supporting
		# foot
		contactModel = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
		for i in supportFootIds:
			xref = crocoddyl.FrameTranslation(i, np.array([0., 0., 0.]))
			supportContactModel = \
				crocoddyl.ContactModel3D(self.state, xref, self.actuation.nu, np.matrix([0., 50.]).T)
			contactModel.addContact(self.rmodel.frames[i].name + "_contact", supportContactModel)

		# Creating the cost model for a contact phase
		costModel = crocoddyl.CostModelSum(self.state, self.actuation.nu)

		# Add CoM barrier task to avoid falling to one side
		if comTask is not None:

			# lowerLimit = np.array([-10.,] * 3)
			# upperLimit = np.array([10.,] * 3)

			lowerLimit = comTask[0]
			upperLimit = comTask[1]
			comRef = comTask[2]
			# print(lowerLimit, upperLimit, comRef)
			act_ineq = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lowerLimit, upperLimit))

			comTrack = crocoddyl.CostModelCoMPosition(self.state,act_ineq, comRef, self.actuation.nu)
			costModel.addCost("comTrack", comTrack, 1e6)
		for i in supportFootIds:
			cone = crocoddyl.FrictionCone(self.nsurf, self.mu, 4, False)
			frictionCone = crocoddyl.CostModelContactFrictionCone(
				self.state, crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(cone.lb, cone.ub)),
				crocoddyl.FrameFrictionCone(i, cone), self.actuation.nu)
			costModel.addCost(self.rmodel.frames[i].name + "_frictionCone", frictionCone, 1e1)
		if swingFootTask is not None:
			for i in swingFootTask:
				xref = crocoddyl.FrameTranslation(i.frame, i.oMf.translation)
				footTrack = crocoddyl.CostModelFrameTranslation(self.state, xref, self.actuation.nu)
				costModel.addCost(self.rmodel.frames[i.frame].name + "_footTrack", footTrack, 1e6)
		if torsoTask is not None:
			torsoRotationFrame = crocoddyl.FrameRotation(self.torsoId, torsoTask)
			torsoTrack = crocoddyl.CostModelFrameRotation(self.state, torsoRotationFrame, self.actuation.nu)
			costModel.addCost(self.rmodel.frames[torsoRotationFrame.frame].name+'_torsoTrack',torsoTrack,1e5)
		
		# Add collision task
		if collisionTask is not None:
			for i in collisionTask:
				lowerLimit = np.array([-5.,] * 3)
				upperLimit = np.array([5.,] * 3)
				lowerLimit[2] = 0  # inf position lower limit
				#upperLimit[:3] = 10  # inf velocity upper limit

				act_ineq = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lowerLimit, upperLimit))
				fp = crocoddyl.FramePlacement(i, pinocchio.SE3(np.eye(3), zero(3)))
				xref = crocoddyl.FrameTranslation(fp.frame, zero(3))
				collisionAvoid = crocoddyl.CostModelFrameTranslation(self.state, act_ineq, xref, self.actuation.nu)
				costModel.addCost(self.rmodel.frames[fp.frame].name + "_collision", collisionAvoid, 1e6)


		# if isTerminal:
		#     reference_state = self.rmodel.defaultState.copy()

		#     reference_state[8] = np.pi/4. # left hip roll
		#     reference_state[13] += np.pi/2 # left shoulder pitch
		#     reference_state[14] += np.pi/3 # left shoulder roll
		#     reference_state[16] += np.pi/2+np.pi/6 # left elbow

		#     reference_state[17] += np.pi/3 # right hip yaw


		#     # reference_state[10] =  0.
		#     # reference_state[11] =  0.
		#     # reference_state[12] =  0.
		#     # reference_state[19] -= np.pi/4 # right knee
		#     # 0-6
		#     # left leg
		#     # left arm
		#     # right leg
		#     # right arm

		#     stateWeights = np.array([0.] * 3 + [0.] * 3 
		#                             + [20.] +[5000.]+[20.] + [500.]+[50.]*2 
		#                             + [1.]+[10.]+[1.] + [1.]
		#                             + [100.]+[100.] +[0.01]+ [0.01]+[0.01] *2
		#                             + [0.01]*4
		#                             + [10] * self.state.nv)
		#     stateReg = crocoddyl.CostModelState(self.state,
		#                                         crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeights**2).T),
		#                                         reference_state, self.actuation.nu)
		# else:
		#     reference_state = self.rmodel.defaultState.copy()

		#     reference_state[20] -= np.pi/5 # right knee
		#     reference_state[13] += np.pi/2 # left shoulder pitch
			
		#     stateWeights = np.array([0] * 3 + [5.] * 3 
		#                             + [1.]*3+[1.] *3 
		#                             + [10.]+[1.]*3
		#                             + [0.01]+[0.01]+ [0.01]+[100.] +[0.01]*2 
		#                             + [1.] * 4 
		#                             + [10.] * self.state.nv)
		stateReg = crocoddyl.CostModelState(self.state,
											crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeight[1]**2).T),
											stateWeight[0], self.actuation.nu)
		
		costModel.addCost("stateReg", stateReg, 1e1)
		ctrlReg = crocoddyl.CostModelControl(self.state, self.actuation.nu)
		costModel.addCost("ctrlReg", ctrlReg, 1e0)
		#exit()
		lb = np.vstack([self.state.lb[1:self.state.nv + 1], self.state.lb[-self.state.nv:]])
		ub = np.vstack([self.state.ub[1:self.state.nv + 1], self.state.ub[-self.state.nv:]])
		# print(lb, ub)
		stateBounds = crocoddyl.CostModelState(
		    self.state, crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lb, ub)),
		    0 * self.rmodel.defaultState, self.actuation.nu)
		costModel.addCost("stateBounds", stateBounds, 1e6)

		# Creating the action model for the KKT dynamics with simpletic Euler
		# integration scheme
		dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(self.state, self.actuation, contactModel,
																	 costModel, 0., True)
		model = crocoddyl.IntegratedActionModelEuler(dmodel, timeStep)
		return model
	
	def createSwingFootModel1(self, timeStep, supportFootIds, stateWeight, comTask=None, swingFootTask=None, torsoTask =None, collisionTask = None, isTerminal = False):
		""" Action model for a swing foot phase.

		:param timeStep: step duration of the action model
		:param supportFootIds: Ids of the constrained feet
		:param comTask: CoM task
		:param swingFootTask: swinging foot task
		:return action model for a swing foot phase
		"""
		# Creating a 6D multi-contact model, and then including the supporting
		# foot
		contactModel = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
		for i in supportFootIds:
			Mref = crocoddyl.FramePlacement(i, pinocchio.SE3.Identity())
			supportContactModel = \
				crocoddyl.ContactModel6D(self.state, Mref, self.actuation.nu, np.matrix([0., 0.]).T)
			contactModel.addContact(self.rmodel.frames[i].name + "_contact", supportContactModel)
			# xref = crocoddyl.FrameTranslation(i, np.array([0., 0., 0.]))
			# supportContactModel = \
			#     crocoddyl.ContactModel3D(self.state, xref, self.actuation.nu, np.matrix([0., 50.]).T)
			# contactModel.addContact(self.rmodel.frames[i].name + "_contact", supportContactModel)
			'''
			xref = crocoddyl.FrameTranslation(i, np.array([0., 0., 0.]))
			supportContactModel = \
				crocoddyl.ContactModel3D(self.state, xref, self.actuation.nu, np.matrix([0., 50.]).T)
			contactModel.addContact(self.rmodel.frames[i].name + "_contact", supportContactModel)
			'''
		# Creating the cost model for a contact phase
		costModel = crocoddyl.CostModelSum(self.state, self.actuation.nu)

		# Add CoM barrier task to avoid falling to one side
		if comTask is not None:

			lowerLimit = np.array([-10.,] * 3)
			upperLimit = np.array([10.,] * 3)

			lowerLimit[1] = comTask[0]
			upperLimit[1] = comTask[1]
			
			comRef = comTask[2]
			# print(lowerLimit, upperLimit, comRef)
			act_ineq = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lowerLimit, upperLimit))

			comTrack = crocoddyl.CostModelCoMPosition(self.state,act_ineq, comRef, self.actuation.nu)
			costModel.addCost("comTrack", comTrack, 1e1)
		for i in supportFootIds:
			cone = crocoddyl.FrictionCone(self.nsurf, self.mu, 4, False)
			frictionCone = crocoddyl.CostModelContactFrictionCone(
				self.state, crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(cone.lb, cone.ub)),
				crocoddyl.FrameFrictionCone(i, cone), self.actuation.nu)
			costModel.addCost(self.rmodel.frames[i].name + "_frictionCone", frictionCone, 1e1)
		if swingFootTask is not None:
			for i in swingFootTask:
				xref = crocoddyl.FrameTranslation(i.frame, i.oMf.translation)
				footTrack = crocoddyl.CostModelFrameTranslation(self.state, xref, self.actuation.nu)
				costModel.addCost(self.rmodel.frames[i.frame].name + "_footTrack", footTrack, 1e6)
		if torsoTask is not None:
			torsoRotationFrame = crocoddyl.FrameRotation(self.torsoId, torsoTask)
			torsoTrack = crocoddyl.CostModelFrameRotation(self.state, torsoRotationFrame, self.actuation.nu)
			costModel.addCost(self.rmodel.frames[torsoRotationFrame.frame].name+'_torsoTrack',torsoTrack,1e6)
		
		# Add collision task
		if collisionTask is not None:
			for i in collisionTask:
				lowerLimit = np.array([-10.,] * 3)
				upperLimit = np.array([10.,] * 3)
				lowerLimit[2] = 0.1  # inf position lower limit
				#upperLimit[:3] = 10  # inf velocity upper limit

				act_ineq = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lowerLimit, upperLimit))
				fp = crocoddyl.FramePlacement(i, pinocchio.SE3(np.eye(3), zero(3)))
				xref = crocoddyl.FrameTranslation(fp.frame, zero(3))
				collisionAvoid = crocoddyl.CostModelFrameTranslation(self.state, act_ineq, xref, self.actuation.nu)
				costModel.addCost(self.rmodel.frames[fp.frame].name + "_collision", collisionAvoid, 1e6)
		# region
		# if isTerminal:
		#     reference_state = self.rmodel.defaultState.copy()

		#     reference_state[8] = np.pi/4. # left hip roll
		#     reference_state[13] += np.pi/2 # left shoulder pitch
		#     reference_state[14] += np.pi/3 # left shoulder roll
		#     reference_state[16] += np.pi/2+np.pi/6 # left elbow

		#     reference_state[17] += np.pi/3 # right hip yaw


		#     # reference_state[10] =  0.
		#     # reference_state[11] =  0.
		#     # reference_state[12] =  0.
		#     # reference_state[19] -= np.pi/4 # right knee
		#     # 0-6
		#     # left leg
		#     # left arm
		#     # right leg
		#     # right arm

		#     stateWeights = np.array([0.] * 3 + [0.] * 3 
		#                             + [20.] +[5000.]+[20.] + [500.]+[50.]*2 
		#                             + [1.]+[10.]+[1.] + [1.]
		#                             + [100.]+[100.] +[0.01]+ [0.01]+[0.01] *2
		#                             + [0.01]*4
		#                             + [10] * self.state.nv)
		#     stateReg = crocoddyl.CostModelState(self.state,
		#                                         crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeights**2).T),
		#                                         reference_state, self.actuation.nu)
		# else:

		# reference_state = self.rmodel.defaultState.copy()
		# reference_state[13] += np.pi/6 # left shoulder pitch
		# #reference_state[14] += np.pi/3 # left shoulder roll
		# reference_state[16] += np.pi/2+np.pi/6 # left elbow

		# reference_state[20] -= np.pi/4 # right knee
		# #reference_state[8] += np.pi/4 # left hip roll
		# reference_state[18] -=np.pi/6 # right hip roll

		# stateWeights = np.array([0] * 3 + [5.] * 3 
		#                         + [0.01]*1+[1000.]+[0.01]+[1.] *3 
		#                         + [1000.]*4
		#                         + [0.01]+[1000.]+ [0.01]+[1.] +[0.01]*2 
		#                         + [1.] * 4 
		#                         + [20.] * self.state.nv)
		# print(s.stateWeights)
		# print(stateWeights)
		# exit()
		# endregion

		stateReg = crocoddyl.CostModelState(self.state,
											crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeight[1]**2).T),
											stateWeight[0], self.actuation.nu)
		
		costModel.addCost("stateReg", stateReg, 1e1)
		ctrlReg = crocoddyl.CostModelControl(self.state, self.actuation.nu)
		costModel.addCost("ctrlReg", ctrlReg, 1e-1)

		# region
		#exit()
		# lb = np.vstack([self.state.lb[1:self.state.nv + 1], self.state.lb[-self.state.nv:]])
		# ub = np.vstack([self.state.ub[1:self.state.nv + 1], self.state.ub[-self.state.nv:]])
		# stateBounds = crocoddyl.CostModelState(
		#     self.state, crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lb, ub)),
		#     0 * self.rmodel.defaultState, self.actuation.nu)
		# costModel.addCost("stateBounds", stateBounds, 1e1)
		# endregion

		# Creating the action model for the KKT dynamics with simpletic Euler
		# integration scheme
		dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(self.state, self.actuation, contactModel,
																	 costModel, 0., True)
		model = crocoddyl.IntegratedActionModelEuler(dmodel, timeStep)
		return model

	def createFootSwitchModel(self, supportFootIds, swingFootTask, pseudoImpulse=True):
		""" Action model for a foot switch phase.

		:param supportFootIds: Ids of the constrained feet
		:param swingFootTask: swinging foot task
		:param pseudoImpulse: true for pseudo-impulse models, otherwise it uses the impulse model
		:return action model for a foot switch phase
		"""
		if pseudoImpulse:
			return self.createPseudoImpulseModel(supportFootIds, swingFootTask)
		else:
			return self.createImpulseModel(supportFootIds, swingFootTask)

	def createPseudoImpulseModel(self, supportFootIds, swingFootTask):
		""" Action model for pseudo-impulse models.

		A pseudo-impulse model consists of adding high-penalty cost for the contact velocities.
		:param supportFootIds: Ids of the constrained feet
		:param swingFootTask: swinging foot task
		:return pseudo-impulse differential action model
		"""

		# Creating a 6D multi-contact model, and then including the supporting
		# foot
		contactModel = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
		for i in supportFootIds:
			xref = crocoddyl.FrameTranslation(i, np.array([0., 0., 0.]))
			supportContactModel = crocoddyl.ContactModel3D(self.state, xref, self.actuation.nu, np.matrix([0., 50.]).T)
			contactModel.addContact(self.rmodel.frames[i].name + "_contact", supportContactModel)

		# Creating the cost model for a contact phase
		costModel = crocoddyl.CostModelSum(self.state, self.actuation.nu)
		for i in supportFootIds:
			cone = crocoddyl.FrictionCone(self.nsurf, self.mu, 4, False)
			frictionCone = crocoddyl.CostModelContactFrictionCone(
				self.state, crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(cone.lb, cone.ub)),
				crocoddyl.FrameFrictionCone(i, cone), self.actuation.nu)
			costModel.addCost(self.rmodel.frames[i].name + "_frictionCone", frictionCone, 1e1)
		if swingFootTask is not None:
			for i in swingFootTask:
				xref = crocoddyl.FrameTranslation(i.frame, i.oMf.translation)
				vref = crocoddyl.FrameMotion(i.frame, pinocchio.Motion.Zero())
				footTrack = crocoddyl.CostModelFrameTranslation(self.state, xref, self.actuation.nu)
				impulseFootVelCost = crocoddyl.CostModelFrameVelocity(self.state, vref, self.actuation.nu)
				
				costModel.addCost(self.rmodel.frames[i.frame].name + "_footTrack", footTrack, 1e6)
				costModel.addCost(self.rmodel.frames[i.frame].name + "_impulseVel", impulseFootVelCost, 1e6)

		stateWeights = np.array([0] * 3 + [10000.] * 3 + [0.01] * (self.state.nv - 6) + [10] * self.state.nv)
		stateReg = crocoddyl.CostModelState(self.state,
											crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeights**2).T),
											self.rmodel.defaultState, self.actuation.nu)
		ctrlReg = crocoddyl.CostModelControl(self.state, self.actuation.nu)
		costModel.addCost("stateReg", stateReg, 1e1)
		costModel.addCost("ctrlReg", ctrlReg, 1e-3)

		# Creating the action model for the KKT dynamics with simpletic Euler
		# integration scheme
		dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(self.state, self.actuation, contactModel,
																	 costModel, 0., True)
		model = crocoddyl.IntegratedActionModelEuler(dmodel, 0.)
		return model

	def createImpulseModel(self, supportFootIds, swingFootTask):
		""" Action model for impulse models.

		An impulse model consists of describing the impulse dynamics against a set of contacts.
		:param supportFootIds: Ids of the constrained feet
		:param swingFootTask: swinging foot task
		:return impulse action model
		"""
		# Creating a 6D multi-contact model, and then including the supporting foot
		impulseModel = crocoddyl.ImpulseModelMultiple(self.state)
		for i in supportFootIds:
			supportContactModel = crocoddyl.ImpulseModel3D(self.state, i)
			impulseModel.addImpulse(self.rmodel.frames[i].name + "_impulse", supportContactModel)

		# Creating the cost model for a contact phase
		costModel = crocoddyl.CostModelSum(self.state, 0)
		if swingFootTask is not None:
			for i in swingFootTask:
				xref = crocoddyl.FrameTranslation(i.frame, i.oMf.translation)
				footTrack = crocoddyl.CostModelFrameTranslation(self.state, xref, 0)
				costModel.addCost(self.rmodel.frames[i.frame].name + "_footTrack", footTrack, 1e6)
		stateWeights = np.array([1.] * 6 + [0.1] * (self.rmodel.nv - 6) + [10] * self.rmodel.nv)
		stateReg = crocoddyl.CostModelState(self.state,
											crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeights**2).T),
											self.rmodel.defaultState, 0)
		costModel.addCost("stateReg", stateReg, 1e1)

		# Creating the action model for the KKT dynamics with simpletic Euler
		# integration scheme
		model = crocoddyl.ActionModelImpulseFwdDynamics(self.state, impulseModel, costModel, 0.55)
		# model = crocoddyl.ActionModelImpulseFwdDynamics(self.state, impulseModel, costModel)
		return model

def get_cmap(n, name='hsv'):
	import matplotlib.pyplot as plt
	'''
	Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
	RGB color; the keyword argument name must be a standard mpl colormap name.
	'''
	return plt.cm.get_cmap(name, n)

def plotSolution(solver, bounds=True, figIndex=1, figTitle="", show=True):
	import matplotlib.pyplot as plt
	xs, us = [], []
	if bounds:
		us_lb, us_ub = [], []
		xs_lb, xs_ub = [], []
	if isinstance(solver, list):
		rmodel = solver[0].problem.runningModels[0].state.pinocchio
		for s in solver:
			xs.extend(s.xs[:-1])
			us.extend(s.us)
			if bounds:
				models = s.problem.runningModels + [s.problem.terminalModel]
				for m in models:
					us_lb += [m.u_lb]
					us_ub += [m.u_ub]
					xs_lb += [m.state.lb]
					xs_ub += [m.state.ub]
	else:
		rmodel = solver.problem.runningModels[0].state.pinocchio
		xs, us = solver.xs, solver.us
		if bounds:
			models = solver.problem.runningModels + [solver.problem.terminalModel]
			for m in models:
				us_lb += [m.u_lb]
				us_ub += [m.u_ub]
				xs_lb += [m.state.lb]
				xs_ub += [m.state.ub]

	# Getting the state and control trajectories
	nx, nq, nu = xs[0].shape[0], rmodel.nq, us[0].shape[0]
	X = [0.] * nx
	U = [0.] * nu
	if bounds:
		U_LB = [0.] * nu
		U_UB = [0.] * nu
		X_LB = [0.] * nx
		X_UB = [0.] * nx
	for i in range(nx):
		X[i] = [np.asscalar(x[i]) for x in xs]
		if bounds:
			X_LB[i] = [np.asscalar(x[i]) for x in xs_lb]
			X_UB[i] = [np.asscalar(x[i]) for x in xs_ub]
	for i in range(nu):
		U[i] = [np.asscalar(u[i]) if u.shape[0] != 0 else 0 for u in us]
		if bounds:
			U_LB[i] = [np.asscalar(u[i]) if u.shape[0] != 0 else np.nan for u in us_lb]
			U_UB[i] = [np.asscalar(u[i]) if u.shape[0] != 0 else np.nan for u in us_ub]

	# Plotting the joint positions, velocities and torques
	# Paper formating
	cycol = cycle('bgrcmk')
	plt.rcParams["font.family"] = "Times New Roman"
	plt.rcParams.update({'font.size': 12})
	plt.figure(figIndex)
	plt.suptitle(figTitle)
	legJointNames = ['Left hip yaw', 'Left hip roll', 'Left hip pitch','Left knee','Left ankle pitch','Left ankle roll',\
	'Right hip yaw', 'Right hip roll', 'Right hip pitch','Right knee','Right ankle pitch','Right ankle roll']
	# L foot
	plt.figure()
	#plt.title('Joint position (rad)')
	N=12
	cmap = get_cmap(N)
	[plt.plot(np.arange(102)*0.001, X[k],'-', label=legJointNames[i], c=cycol.next()) for i, k in enumerate(range(7, 13))]
	[plt.plot(np.arange(102)*0.001, X[k],'--' , label=legJointNames[i+6], c=cycol.next()) for i, k in enumerate(range(13, 19))]
	if bounds:
		[plt.plot(X_LB[k], '--r') for i, k in enumerate(range(7, 19))]
		[plt.plot(X_UB[k], '--r') for i, k in enumerate(range(7, 19))]
	# Paper formating
	ax = plt.subplot(1, 1, 1)
	plt.ylim(-2.5, 2.5)
	ax.yaxis.set_major_locator(MultipleLocator(0.5))
	plt.xlim(0, 0.1)
	ax.xaxis.set_major_locator(MultipleLocator(0.02))
	
	ax.spines['top'].set_visible(False)
	ax.spines['right'].set_visible(False)
	box = ax.get_position()
	ax.set_position([box.x0, box.y0 + box.height * 0.1,
					box.width, box.height * 0.9])

	# Put a legend below current axis
	ax.legend(loc='upper center', bbox_to_anchor=(0.45, -0.15),
			fancybox=True, shadow=False, ncol=3)
	
	plt.xlabel('$\it{t}$ (s)')
	plt.ylabel('Joint position (rad)')
	plt.savefig('../result/full_joint_position.png',bbox_inches='tight',pad_inches = 0, dpi = 300)

	plt.figure()
	#plt.title('joint velocity [rad/s]')
	[plt.plot(np.arange(102)*0.001, X[k],'-', label=legJointNames[i], c=cycol.next()) for i, k in enumerate(range(nq + 6, nq + 12))]
	[plt.plot(np.arange(102)*0.001, X[k],'--' , label=legJointNames[i+6], c=cycol.next()) for i, k in enumerate(range(nq + 12, nq + 18))]
	#[plt.plot(X[k], label=legJointNames[i]) for i, k in enumerate(range(nq + 6, nq + 12))]
	if bounds:
		[plt.plot(X_LB[k], '--r') for i, k in enumerate(range(nq + 6, nq + 12))]
		[plt.plot(X_UB[k], '--r') for i, k in enumerate(range(nq + 6, nq + 12))]
	ax = plt.subplot(1, 1, 1)
	plt.ylim(-25, 20)
	ax.yaxis.set_major_locator(MultipleLocator(5))
	plt.xlim(0, 0.1)
	ax.xaxis.set_major_locator(MultipleLocator(0.02))
	ax.spines['top'].set_visible(False)
	ax.spines['right'].set_visible(False)
	box = ax.get_position()
	ax.set_position([box.x0, box.y0 + box.height * 0.1,
					box.width, box.height * 0.9])

	# Put a legend below current axis
	ax.legend(loc='upper center', bbox_to_anchor=(0.45, -0.15),
			fancybox=True, shadow=False, ncol=3)
	plt.xlabel('$\it{t}$ (s)')
	plt.ylabel('Joint velocity (rad/s)')
	plt.savefig('../result/full_joint_velocity.png',bbox_inches='tight',pad_inches = 0, dpi = 300)


	plt.figure()
	#plt.title('joint torque [Nm]')
	[plt.plot(np.arange(102)*0.001, U[k],'-', label=legJointNames[i], c=cycol.next()) for i, k in enumerate(range(0, 6))]
	[plt.plot(np.arange(102)*0.001, U[k],'--' , label=legJointNames[i+6], c=cycol.next()) for i, k in enumerate(range(6, 12))]
	
	#[plt.plot(U[k], label=legJointNames[i]) for i, k in enumerate(range(0, 6))]
	if bounds:
		[plt.plot(U_LB[k], '--r') for i, k in enumerate(range(0, 6))]
		[plt.plot(U_UB[k], '--r') for i, k in enumerate(range(0, 6))]
	plt.ylabel('L')
	plt.legend()
	ax = plt.subplot(1, 1, 1)
	plt.ylim(-12, 12)
	ax.yaxis.set_major_locator(MultipleLocator(2))
	plt.xlim(0, 0.1)
	ax.xaxis.set_major_locator(MultipleLocator(0.02))
	ax.spines['top'].set_visible(False)
	ax.spines['right'].set_visible(False)
	box = ax.get_position()
	ax.set_position([box.x0, box.y0 + box.height * 0.1,
					box.width, box.height * 0.9])

	# Put a legend below current axis
	ax.legend(loc='upper center', bbox_to_anchor=(0.45, -0.15),
			fancybox=True, shadow=False, ncol=3)
	plt.xlabel('$\it{t}$ (s)')
	plt.ylabel(r'Joint torque (N$\cdot$m)')
	plt.savefig('../result/full_joint_torque.png',bbox_inches='tight',pad_inches = 0, dpi = 300)

	# R foot
	plt.subplot(4, 3, 4)
	[plt.plot(X[k], label=legJointNames[i]) for i, k in enumerate(range(13, 19))]
	if bounds:
		[plt.plot(X_LB[k], '--r') for i, k in enumerate(range(13, 19))]
		[plt.plot(X_UB[k], '--r') for i, k in enumerate(range(13, 19))]
	plt.ylabel('R')
	plt.legend()
	plt.subplot(4, 3, 5)
	[plt.plot(X[k], label=legJointNames[i]) for i, k in enumerate(range(nq + 12, nq + 18))]
	if bounds:
		[plt.plot(X_LB[k], '--r') for i, k in enumerate(range(nq + 12, nq + 18))]
		[plt.plot(X_UB[k], '--r') for i, k in enumerate(range(nq + 12, nq + 18))]
	plt.ylabel('R')
	plt.legend()
	plt.subplot(4, 3, 6)
	[plt.plot(U[k], label=legJointNames[i]) for i, k in enumerate(range(6, 12))]
	if bounds:
		[plt.plot(U_LB[k], '--r') for i, k in enumerate(range(6, 12))]
		[plt.plot(U_UB[k], '--r') for i, k in enumerate(range(6, 12))]
	plt.ylabel('R')
	plt.legend()
	plt.xlabel('knots')

	plt.figure(figIndex + 1)
	plt.suptitle(figTitle)
	if show:
		plt.show()